#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::net::IpAddr;
use core::ptr::addr_of_mut;
use core::slice;
use core::str::FromStr;

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_net::{Ipv4Cidr, Stack, StackResources};
use embassy_net_nrf91::context::Status;
use embassy_net_nrf91::{Runner, State, TraceBuffer, TraceReader, context};
use embassy_nrf::buffered_uarte::{self, BufferedUarteTx};
use embassy_nrf::cryptocell_rng::CcRng;
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive};
use embassy_nrf::uarte::Baudrate;
use embassy_nrf::{Peri, bind_interrupts, interrupt, peripherals, uarte};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[interrupt]
fn IPC() {
    embassy_net_nrf91::on_ipc_irq();
}

//bind_interrupts!(struct Irqs {
//    SERIAL0 => buffered_uarte::InterruptHandler<peripherals::SERIAL0>;
//});
//
//#[embassy_executor::task]
//async fn trace_task(mut uart: BufferedUarteTx<'static>, reader: TraceReader<'static>) -> ! {
//    let mut rx = [0u8; 1024];
//    loop {
//        let n = reader.read(&mut rx[..]).await;
//        unwrap!(uart.write_all(&rx[..n]).await);
//    }
//}

#[embassy_executor::task]
async fn modem_task(runner: Runner<'static>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, embassy_net_nrf91::NetDriver<'static>>) -> ! {
    runner.run().await
}

// Configure modem low-power options. All of these must be issued before
// attach (i.e. before context::Control::run -> enable -> CFUN=1), because
// PSM/eDRX/RAI are negotiated with the network during attach.
//
// Verify exact syntax against nrf91x1_cellular_at_commands_v1.4.pdf — some
// operators also override or deny requested timers.
async fn configure_low_power(control: &context::Control<'_>) {
    let mut buf = [0u8; 256];

    // Hint the modem scheduler to prefer power saving over latency/throughput.
    let _ = control.at_command(b"AT%XDATAPRFL=0", &mut buf).await;

    // Request eDRX for LTE-M (AcT=4). Interval "0101" ≈ 20.48 s — longest
    // eDRX cycle most LTE-M networks will grant. Downlink paging latency is
    // bounded by this interval.
    let _ = control.at_command(b"AT+CEDRXS=2,4,\"0101\"", &mut buf).await;

    // Explicitly disable PSM: we keep TCP alive, and PSM makes the device
    // unreachable during sleep which would break downlink and TCP ACKs.
    let _ = control.at_command(b"AT+CPSMS=0", &mut buf).await;

    // Release Assistance Indication: signal to the network that we're done
    // after an uplink so RRC drops immediately instead of waiting out the
    // inactivity timer. Biggest single win for burst-then-idle patterns.
    let _ = control.at_command(b"AT%RAI=1", &mut buf).await;
}

#[embassy_executor::task]
async fn control_task(
    control: &'static context::Control<'static>,
    config: context::Config<'static>,
    stack: Stack<'static>,
) {
    unwrap!(control.configure(&config).await);
    // configure_low_power(control).await;
    unwrap!(
        control
            .run(|status| {
                stack.set_config_v4(status_to_config(status));
            })
            .await
    );
}

fn status_to_config(status: &Status) -> embassy_net::ConfigV4 {
    let Some(IpAddr::V4(addr)) = status.ip else {
        panic!("Unexpected IP address");
    };

    let gateway = match status.gateway {
        Some(IpAddr::V4(addr)) => Some(addr),
        _ => None,
    };

    let mut dns_servers = Vec::new();
    for dns in status.dns.iter() {
        if let IpAddr::V4(ip) = dns {
            unwrap!(dns_servers.push(*ip));
        }
    }

    embassy_net::ConfigV4::Static(embassy_net::StaticConfigV4 {
        address: Ipv4Cidr::new(addr, 32),
        gateway,
        dns_servers,
    })
}

#[embassy_executor::task]
async fn blink_task(pin: Peri<'static, AnyPin>) {
    let mut led = Output::new(pin, Level::Low, OutputDrive::Standard);
    loop {
        led.set_high();
        Timer::after_millis(1000).await;
        led.set_low();
        Timer::after_millis(1000).await;
    }
}

unsafe extern "C" {
    static __start_ipc: u8;
    static __end_ipc: u8;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("Hello World!");

    spawner.spawn(unwrap!(blink_task(p.P0_02.into())));

    let ipc_mem = unsafe {
        let ipc_start = &__start_ipc as *const u8 as *mut MaybeUninit<u8>;
        let ipc_end = &__end_ipc as *const u8 as *mut MaybeUninit<u8>;
        let ipc_len = ipc_end.offset_from(ipc_start) as usize;
        slice::from_raw_parts_mut(ipc_start, ipc_len)
    };

    //    static mut TRACE_BUF: [u8; 4096] = [0u8; 4096];
    //    let mut config = uarte::Config::default();
    //    config.baudrate = Baudrate::BAUD1M;
    //    let uart = BufferedUarteTx::new(
    //        //let trace_uart = BufferedUarteTx::new(
    //        unsafe { peripherals::SERIAL0::steal() },
    //        unsafe { peripherals::P0_01::steal() },
    //        Irqs,
    //        //unsafe { peripherals::P0_14::steal() },
    //        config,
    //        unsafe { &mut *addr_of_mut!(TRACE_BUF) },
    //    );

    static STATE: StaticCell<State> = StaticCell::new();
    //    static TRACE: StaticCell<TraceBuffer> = StaticCell::new();
    let (device, control, runner) = embassy_net_nrf91::new(STATE.init(State::new()), ipc_mem).await;
    spawner.spawn(unwrap!(modem_task(runner)));
    //    spawner.spawn(unwrap!(trace_task(uart, tracer)));

    let config = embassy_net::Config::default();

    // Generate random seed.
    let mut rng = CcRng::new_blocking(p.CC_RNG);
    let seed = rng.blocking_next_u64();

    // Init network stack
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(device, config, RESOURCES.init(StackResources::<2>::new()), seed);

    spawner.spawn(unwrap!(net_task(runner)));

    static CONTROL: StaticCell<context::Control<'static>> = StaticCell::new();
    let control = CONTROL.init(context::Control::new(control, 0).await);

    spawner.spawn(unwrap!(control_task(
        control,
        context::Config {
            apn: b"iot.nat.es",
            auth_prot: context::AuthProt::Pap,
            auth: Some((b"orange", b"orange")),
            pin: None,
        },
        stack
    )));

    stack.wait_config_up().await;

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        // Keep NAT mappings alive while RRC is idle. 4 min is usually safe;
        // tighten if your operator's TCP NAT timeout is shorter.
        socket.set_keep_alive(Some(Duration::from_secs(240)));

        info!("Connecting...");
        let host_addr = embassy_net::Ipv4Address::from_str("45.79.112.203").unwrap();
        if let Err(e) = socket.connect((host_addr, 4242)).await {
            warn!("connect error: {:?}", e);
            Timer::after_secs(10).await;
            continue;
        }
        info!("Connected to {:?}", socket.remote_endpoint());

        let msg = b"Hello world!\n";
        for _ in 0..10 {
            if let Err(e) = socket.write_all(msg).await {
                warn!("write error: {:?}", e);
                break;
            }
            info!("txd: {}", core::str::from_utf8(msg).unwrap());
            Timer::after_secs(1).await;
        }
        Timer::after_secs(60).await;
    }
}
