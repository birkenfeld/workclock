#![no_main]
#![no_std]

use cortex_m::peripheral::syst::SystClkSource;
use stm32g4xx_hal as dev;
use dev::stm32g4::stm32g431 as pac;
use stm32g4xx_hal::prelude::*;
use stm32g4xx_hal::rcc::{Config, SysClockSrc};
use stm32g4xx_hal::syscfg::SysCfgExt;
use stm32g4xx_hal::gpio::{ExtiPin, SignalEdge};
use ws2812_spi::prerendered::Ws2812;
use smart_leds::{RGB8, SmartLedsWrite, brightness, gamma};

use defmt_rtt as _; // global logger
use panic_probe as _;

const HSI_TRIM: u8 = 25;
const BRIGHTNESS: u8 = 40;

#[macro_use]
mod regutil;

mod pins {
    pub use crate::dev::gpio::{gpioa::*, gpiob::*, gpioc::*};
}

mod mode {
    use crate::dev::gpio::*;
    pub type Alt5 = Alternate<5>;
    pub type Out = Output<PushPull>;
    pub type In = Input<PullDown>;
}

pub struct Clock {
    current: u64,
    started: u64,
}

const OFF: RGB8 = RGB8::new(0, 0, 0);
const RED: RGB8 = RGB8::new(255, 0, 0);
const GREEN: RGB8 = RGB8::new(0, 255, 0);

impl Clock {
    fn start(&mut self) {
        if self.current > 2 && self.started < self.current - 2 {
            defmt::info!("clock reset");
            self.started = self.current;
        }
    }

    fn tick(&mut self) -> [RGB8; 24] {
        let mut buf = [
            // 0-11  first 6 hours
            RED, RED, RED, RED, RED, RED, RED, RED, RED, RED, RED, RED,
            // 12    lunch
            GREEN,
            // 13-16 6-8 hours
            RED, RED, RED, RED,
            // 17    coffee
            GREEN,
            // 18-21 8-10 hours
            RED, RED, RED, RED,
            // 22-23 to indicate activity (seconds)
            OFF, OFF
        ];

        self.current += 1;

        let elapsed = (self.current - self.started) / 2;
        let minutes = elapsed / 60;
        const H: u64 = 60;

        if self.current % 2 == 0 {
            defmt::info!("tick: {}", elapsed);
        }

        if self.started == 0 || minutes >= 10*H + 55 + 15 {
            // overtime, switch off
            self.started = 0;
            return [OFF; 24];
        }

        // remove the worked hours
        for i in 0..12 {
            if minutes >= 30 + i as u64*30 {
                buf[i] = OFF;
            }
        }
        if minutes >= 6*H + 40 {
            buf[12] = OFF;
        } else if minutes >= 6*H {
            // blink during break
            if elapsed % 2 == 1 {
                buf[12] = OFF;
            }
        }
        for i in 0..4 {
            if minutes >= 6*H + 40 + 30 + i as u64*30 {
                buf[13+i] = OFF;
            }
        }
        if minutes >= 8*H + 55 {
            buf[17] = OFF;
        } else if minutes >= 8*H + 40 {
            // blink during break
            if elapsed % 2 == 1 {
                buf[17] = OFF;
            }
        }
        for i in 0..4 {
            if minutes >= 8*H + 55 + 30 + i as u64*30 {
                buf[18+i] = OFF;
            }
        }

        if minutes >= 10*H + 55 {
            // overtime, blink to indicate
            if self.current % 2 == 1 {
                buf = [RED; 24];
            }
        }

        buf
    }
}

static mut BUF: [u8; 24*12+20] = [0; 24*12+20];

#[rtic::app(device = crate::pac, peripherals = true)]
mod app {
    use crate::*;

    #[shared]
    struct Resources {
        state: [RGB8; 24],
        clock: Clock,
    }

    #[local]
    struct Local {
        leds: Ws2812<'static, dev::spi::Spi<dev::stm32::SPI1,
                                   (pins::PB3<mode::Alt5>,
                                    pins::PB4<mode::Alt5>,
                                    pins::PB5<mode::Alt5>)>>,
        button: pins::PA0<mode::In>,
        clkout: pins::PA8<mode::Out>,
    }

    #[init]
    fn init(cx: init::Context) -> (Resources, Local, init::Monotonics) {
        let mut peri = cx.device;

        modif!(RCC.icscr: hsitrim = HSI_TRIM);

        let rcc_config = Config::new(SysClockSrc::PLL);
        let mut syscfg = peri.SYSCFG.constrain();
        let mut rcc = peri.RCC.freeze(rcc_config);
        let gpioa = peri.GPIOA.split(&mut rcc);
        let gpiob = peri.GPIOB.split(&mut rcc);

        defmt::info!("init with sysclk {} MHz", rcc.clocks.sys_clk.0 / 1000000);

        let mut user_led = gpiob.pb8.into_push_pull_output();
        user_led.set_low().unwrap();

        let mut button = gpioa.pa0.into_pull_down_input();
        button.make_interrupt_source(&mut syscfg);
        button.trigger_on_edge(&mut peri.EXTI, SignalEdge::Rising);
        button.enable_interrupt(&mut peri.EXTI);

        let clkout = gpioa.pa8.into_push_pull_output();

        let sclk = gpiob.pb3.into_alternate();
        let miso = gpiob.pb4.into_alternate();
        let mosi = gpiob.pb5.into_alternate();

        let spi = dev::spi::Spi::spi1(peri.SPI1, (sclk, miso, mosi),
                                      ws2812_spi::MODE,
                                      3.mhz(), &mut rcc);
        modif!(GPIOB.otyper: ot5 = true);
        let leds = Ws2812::new(spi, unsafe { &mut BUF });

        let mut syst = cx.core.SYST;
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(rcc.clocks.sys_clk.0);
        syst.enable_counter();
        syst.enable_interrupt();

        (Resources { state: [OFF; 24], clock: Clock { current: 0, started: 0 } },
         Local { leds, button, clkout },
         init::Monotonics())
    }

    #[idle(shared = [state], local = [leds])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            let state = cx.shared.state.lock(|v| *v);
            cortex_m::interrupt::free(|_| {
                cx.local.leds.write(brightness(gamma(state.into_iter()),
                                               BRIGHTNESS)).unwrap();
            });
            cortex_m::asm::delay(10_000);
        }
    }

    #[task(binds = EXTI0, shared = [clock], local = [button])]
    fn button(mut cx: button::Context) {
        cx.shared.clock.lock(|v| v.start());
        cx.local.button.clear_interrupt_pending_bit();
    }

    #[task(binds = SysTick, shared = [state, clock], local = [clkout])]
    fn systick(mut cx: systick::Context) {
        let state = cx.shared.clock.lock(|v| v.tick());
        cx.shared.state.lock(|v| *v = state);
        // cx.local.clkout.toggle().unwrap();
    }
}
