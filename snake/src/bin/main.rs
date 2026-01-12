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
    timer::*, //use to make duty cycles. and PWM 
};
use esp_println as _;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        info!("FAULT: UNRECOVERABLE_EXCEPTION_DETECTED");
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

fn forward(wheelr1: &mut Output, wheelr2: &mut Output, wheell1: &mut Output, wheell2: &mut Output) {
    wheelr1.set_high();
    wheelr2.set_low();
    wheell1.set_high();
    wheell2.set_low();
}

fn backward(
    wheelr1: &mut Output,
    wheelr2: &mut Output,
    wheell1: &mut Output,
    wheell2: &mut Output,
) {
    wheelr1.set_low();
    wheelr2.set_high();
    wheell1.set_low();
    wheell2.set_high();
}

fn right(wheelr1: &mut Output, wheelr2: &mut Output, wheell1: &mut Output, wheell2: &mut Output) {
    wheell1.set_high();
    wheell2.set_low();
    let mut delay = Delay::new();
    loop {
        wheelr1.set_low();
        wheelr2.set_low();
        delay.delay_millis(800);
        wheelr1.set_high();
        wheelr2.set_low();
        delay.delay_millis(600);
        break;
    }
}

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

    let _gpio7 = Input::new(peripherals.GPIO7, inconfig);
    let _gpio8 = Input::new(peripherals.GPIO8, inconfig);

    let mut delay = Delay::new();

    info!("MOTOR_CONTROL_ACTIVE");

    loop {
        info!("turning right");
        right(&mut gpio9, &mut gpio10, &mut gpio2, &mut gpio3);
        delay.delay_millis(500);

        info!("moving forward");
        forward(&mut gpio9, &mut gpio10, &mut gpio2, &mut gpio3);
        delay.delay_millis(7000);

        info!("moving BACKward");
        backward(&mut gpio9, &mut gpio10, &mut gpio2, &mut gpio3);
        delay.delay_millis(8000);
    }
}
