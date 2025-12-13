#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use esp_hal::{
    delay::Delay,
    gpio::{Input, Io, Pull},
};

use esp_println as _;
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    // generator version: 1.0.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    info!("SYSTEM BOOT SEQUENCE INITIALIZED");
    //esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);
    let mut io = io::IO::new(
        peripherals.GPIO,   // GPIO peripheral
        peripherals.IO_MUX, // IO multiplexer
    );

    let sensor = Input::new(
        io.pins.gpio7, // consumes gpio7
        InputConfig::new().pull(Pull::Up),
    );
    info!("[INIT] Entering control loop");
    let mut delay = Delay::new();

    loop {
        let sensor_state = sensor.is_high();
        if sensor_state {
            info!("NO WALLS DETECTED, A SYSTEM DECISION MUST BE MADE");
        } else {
            info!("there is a wall. continue that route!");
            continue;
        }
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
