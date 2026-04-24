#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{bind_interrupts, pac, peripherals, uarte};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SERIAL0 => uarte::InterruptHandler<peripherals::SERIAL0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.dcdc.regmain = true;
    //    config.debug = embassy_nrf::config::Debug::Disallowed;
    let p = embassy_nrf::init(config);
    drop(p);
    //  let uart = uarte::Uarte::new(p.SERIAL0, p.P0_01, p.P0_02, Irqs, Default::default());
    //  drop(uart);

    let power = pac::POWER_S;
    power
        .ltemodem()
        .forceoff()
        .write(|w| w.set_forceoff(pac::power::vals::Forceoff::HOLD));
    power.tasks_lowpwr().write_value(1);
    //    let clock = pac::CLOCK;
    //    let stat = clock.hfclkstat().read();
    //    clock.tasks_hfclkstop().write_value(1);
    //    let regulators = pac::REGULATORS_S;
    //    regulators.systemoff().write(|w| w.set_systemoff(true));

    /*
    power
        .ltemodem()
        .startn()
        .write(|w| w.set_startn(pac::power::vals::Startn::START));

    */
    loop {
        Timer::after_millis(10000).await;
    }
}
