#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is unsafe with esp_hal types holding buffers."
)]
use defmt::info;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
};
use esp_println as _;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        info!("FAULT: UNRECOVERABLE_EXCEPTION_DETECTED");
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    info!("SUBSYSTEM_INITIALIZATION_COMPLETE");

    let outconfig = OutputConfig::default();
    let inconfig = InputConfig::default();

    let mut gpio9 = Output::new(peripherals.GPIO9, Level::High, outconfig);
    let mut gpio10 = Output::new(peripherals.GPIO10, Level::Low, outconfig);
    let mut gpio2 = Output::new(peripherals.GPIO2, Level::High, outconfig);
    let mut gpio3 = Output::new(peripherals.GPIO3, Level::Low, outconfig);

    let gpio7 = Input::new(peripherals.GPIO7, inconfig);
    let gpio8 = Input::new(peripherals.GPIO8, inconfig);

    let mut delay = Delay::new();

    info!("MOTOR_CONTROL_ACTIVE");
    info!("ENCODER_MONITORING_ACTIVE");
    info!("COMMENCING_OPERATIONAL_LOOP");

    let mut cycle_counter = 0u32;
    let mut direction_forward = true;

    loop {
        let gpio7_state = if gpio7.is_high() { 1 } else { 0 };
        let gpio8_state = if gpio8.is_high() { 1 } else { 0 };

        info!("GPIO7: {} | GPIO8: {}", gpio7_state, gpio8_state);

        cycle_counter += 1;
        if cycle_counter >= 50 {
            cycle_counter = 0;
            direction_forward = !direction_forward;

            if direction_forward {
                gpio9.set_high();
                gpio10.set_low();
                gpio2.set_high();
                gpio3.set_low();
                info!("WHEEL_1: FORWARD");
                info!("WHEEL_2: FORWARD");
            } else {
                gpio9.set_low();
                gpio10.set_high();
                gpio2.set_low();
                gpio3.set_high();
                info!("WHEEL_1: REVERSE");
                info!("WHEEL_2: REVERSE");
            }
        }

        delay.delay_millis(100);
    }
}
