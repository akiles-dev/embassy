#![no_std]
#![no_main]
#![allow(dead_code)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive, Pull, Input, Flex, AnyPin};
use embassy_nrf::config::{Debug, ClockSpeed, HfclkSource, LfclkSource};
use embassy_nrf::pac;
use embassy_nrf::twim;
use embassy_nrf::Peri;
use embassy_nrf::peripherals;
use defmt::unwrap;
use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};
use static_cell::StaticCell;

#[embassy_executor::main]
async fn main(s: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.debug = Debug::NotConfigured;
    //config.clock_speed = ClockSpeed::CK64;
    //config.hfclk_source = HfclkSource::ExternalXtal;
    //config.lfclk_source = LfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);
    //pac::REGULATORS.vregmain().dcdcen().write(|w| w.set_val(true));
    // 
    let i2c_sda = p.P1_05;
    let i2c_scl = p.P1_06;
    // 
    let twi = Twim::new(i2c_scl, i2c_sda, p.SERIAL20);
    let twi = TWI.init(twi);

    info!("init led...");
    let led = RgbLed::new(twi).await;

    let mut scb: cortex_m::peripheral::SCB = unsafe { core::mem::transmute(()) };
    scb.set_sleepdeep();
    scb.set_sleeponexit();
}




pub struct RgbLed {
    _private: (),
}

const ADDR: u8 = 0x30;

const REG_EN_RST: u8 = 0x00;
const REG_FLASH_PERIOD: u8 = 0x01;
const REG_PWM1_TIMER: u8 = 0x02;
const REG_PWM2_TIMER: u8 = 0x03;
const REG_LED_EN: u8 = 0x04;
const REG_TRISE_TFALL: u8 = 0x05;
const REG_LED1: u8 = 0x06;
const REG_LED2: u8 = 0x07;
const REG_LED3: u8 = 0x08;
const REG_LED4: u8 = 0x09;
const REG_MAX: u8 = 0x09;

const LED_COUNT: usize = 4;
static CORR: [u8; LED_COUNT] = [0x80, 0x90, 0x60, 0x80];

static SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static mut VALUES: [u8; LED_COUNT] = [0; LED_COUNT];

fn calc(val: u8, corr: u8) -> u8 {
    ((val as u32) * (val as u32) * (corr as u32) / 65536) as u8
}

static TWI: StaticCell<Twim> = StaticCell::new();

#[embassy_executor::task]
async fn run(twi: &'static Twim) {
    loop {
        twi.with(|twi| {
            let mut en = 0;
            let values = unsafe { VALUES };
            for (i, v) in values.into_iter().enumerate() {
                let _ = twi.blocking_write(ADDR, &[REG_LED1 + i as u8, calc(v, CORR[i])]);
                if v != 0 {
                    en |= 1u8 << (i * 2);
                }
            }
            let _ = twi.blocking_write(ADDR, &[REG_LED_EN, en]);

            let enrst = if en == 0 {
                // Device Enters Shutdown Mode Condition: Either SCL goes low or SDA stops toggling
                0x08
            } else {
                // Device Enters Shutdown Mode Condition: Device always ON
                0x00
            };

            let _ = twi.blocking_write(ADDR, &[REG_EN_RST, enrst]);
        });

        SIGNAL.wait().await;
    }
}

impl RgbLed {
    pub async fn new(twi: &'static Twim) -> Self {
        twi.with(|twi| {
            // ignoring error, the softreset command always causes a nack.
            let _ = twi.blocking_write(ADDR, &[REG_EN_RST, 0x07]);
        });

        //Timer::after(Duration::from_millis(1)).await;

        Spawner::for_current_executor().await.spawn(unwrap!(run(twi)));
        let mut this = Self { _private: () };
        this.set(0, 0, 0, 0);
        this
    }

    pub fn set(&mut self, r: u8, g: u8, b: u8, w: u8) {
        unsafe { VALUES = [w, b, g, r] };
        SIGNAL.signal(());
    }
}
pub type Bus<'a> = embassy_nrf::twim::Twim<'a>;

bind_interrupts!(struct Irqs {
    SERIAL20 => twim::InterruptHandler<peripherals::SERIAL20>;
});
type TWIMBUS = peripherals::SERIAL20;

pub struct Twim {
    inner: RefCell<Inner>,
}
struct Inner {
    scl: Peri<'static, AnyPin>,
    sda: Peri<'static, AnyPin>,

    twim: Peri<'static, TWIMBUS>,
}


impl Twim {
    pub fn new(mut scl: Peri<'static, AnyPin>, mut sda: Peri<'static, AnyPin>, twim: Peri<'static, TWIMBUS>) -> Self {
        {
            // Try to unstick the i2c bus if it's stuck.
            let mut scl = Flex::new(scl.reborrow());
            scl.set_high();
            scl.set_as_input_output(Pull::None, OutputDrive::HighDrive0Disconnect1);
            let mut sda = Flex::new(sda.reborrow());
            sda.set_high();
            sda.set_as_input_output(Pull::None, OutputDrive::HighDrive0Disconnect1);

            if sda.is_low() {
                warn!("SDA stuck low.")
            }
            if scl.is_low() {
                warn!("SCL stuck low.")
            }

            info!("doing start+stop");
            cortex_m::asm::delay(64_000_000 / 100_000 / 2);
            sda.set_low();
            cortex_m::asm::delay(64_000_000 / 100_000 / 2);
            sda.set_high();
            cortex_m::asm::delay(64_000_000 / 100_000 / 2);

            info!("wiggling SCL...");
            for _ in 0..12 {
                scl.set_low();
                cortex_m::asm::delay(64_000_000 / 100_000 / 2);
                scl.set_high();
                cortex_m::asm::delay(64_000_000 / 100_000 / 2);

                if scl.is_low() {
                    warn!("SCL still low while clocking it.")
                }
            }

            if sda.is_low() {
                warn!("SDA still stuck low.")
            }
            if scl.is_low() {
                warn!("SCL still stuck low.")
            }

            info!("doing start+stop");
            cortex_m::asm::delay(64_000_000 / 100_000 / 2);
            sda.set_low();
            cortex_m::asm::delay(64_000_000 / 100_000 / 2);
            sda.set_high();
            cortex_m::asm::delay(64_000_000 / 100_000 / 2);

            if sda.is_low() {
                warn!("SDA STILL stuck low, wtf?")
            }
            if scl.is_low() {
                warn!("SCL STILL stuck low, wtf?")
            }
        }

        Self {
            inner: RefCell::new(Inner { scl, sda, twim }),
        }
    }

    pub fn with<R>(&self, f: impl FnOnce(&mut Bus) -> R) -> R {
        let this = &mut *self.inner.borrow_mut();
        let mut config = twim::Config::default();
        config.frequency = twim::Frequency::K400;
        config.scl_high_drive = true;
        config.sda_high_drive = true;
        let mut buf = [0u8; 256];
        let mut twi = twim::Twim::new(
            this.twim.reborrow(),
            Irqs,
            this.sda.reborrow(),
            this.scl.reborrow(),
            config,
            &mut buf,
        );

        f(&mut twi)
    }
}

const TWI_TIMEOUT: Duration = Duration::from_millis(100);

pub struct HalTwim(pub &'static Twim);

impl embedded_hal::blocking::i2c::Write for HalTwim {
    type Error = embassy_nrf::twim::Error;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.with(|twi| twi.blocking_write_timeout(address, bytes, TWI_TIMEOUT))
    }
}

impl embedded_hal::blocking::i2c::WriteRead for HalTwim {
    type Error = embassy_nrf::twim::Error;

    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.0
            .with(|twi| twi.blocking_write_read_timeout(address, bytes, buffer, TWI_TIMEOUT))
    }
}

impl embedded_hal_1::i2c::ErrorType for HalTwim {
    type Error = embassy_nrf::twim::Error;
}

impl embedded_hal_1::i2c::I2c for HalTwim {
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.0.with(|twi| twi.blocking_read_timeout(address, buffer, TWI_TIMEOUT))
    }

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.with(|twi| twi.blocking_write_timeout(address, bytes, TWI_TIMEOUT))
    }

    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.0
            .with(|twi| twi.blocking_write_read_timeout(address, bytes, buffer, TWI_TIMEOUT))
    }

    fn transaction<'a>(
        &mut self,
        _address: u8,
        _operations: &mut [embedded_hal_1::i2c::Operation<'a>],
    ) -> Result<(), Self::Error> {
        defmt::todo!()
    }
}
