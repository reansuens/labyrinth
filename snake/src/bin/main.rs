#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is unsafe with esp_hal types holding buffers."
)]

use defmt::info;
use esp_hal::{
    clock::cpuclock,
    delay::delay,
    gpio::{input, inputconfig, level, output, outputconfig, pin},
    ledc::{
        channel::{self, channeliface},
        timer::{self, timeriface},
        lsglobalclksource, ledc, lowspeed,
    },
    main,
    time::rate,
    timer::periodictimer, //use to make duty cycles. and pwm
};
use esp_println as _;

#[panic_handler]
fn panic(_: &core::panic::panicinfo) -> ! {
    loop {
        info!("fault: unrecoverable_exception_detected");
    }
}

#[derive(clone, copy)]
enum motorselect {
    right,
    left,
}
#[derive(clone, copy)]
enum direction {
    forward,
    backward,
    brake,
    coast,
}

#[derive(debug, clone)]
struct motorconfig {
    frequency: u32,
    duty_resolution: u8,
}

impl default for motorconfig {
    fn default() -> self {
        self {
            frequency: 15000,
            duty_resolution: 10,
        }
    }
}

struct motorcontroller<'a> {
    mr1: output<'a>,
    mr2: output<'a>,
    ml1: output<'a>,
    ml2: output<'a>,
    pwm_right: channel::channel<'a, lowspeed>,
    pwm_left: channel::channel<'a, lowspeed>,
    max_duty: u32,
}
impl<'a> motorcontroller<'a> {
    fn new(
        mr1: output<'a>,
        mr2: output<'a>,
        ml1: output<'a>,
        ml2: output<'a>,
        pwm_right: channel::channel<'a, lowspeed>,
        pwm_left: channel::channel<'a, lowspeed>,
        config: &motorconfig,
    ) -> self {
        let max_duty = (1u32 << config.duty_resolution) - 1;

        self {
            mr1,
            mr2,
            ml1,
            ml2,
            pwm_right,
            pwm_left,
            max_duty,
        }
    }

    fn set_motor(&mut self, motor: motorselect, speed: u8, direction: direction) {
        let speed_clamped = speed.min(100);
        let duty: u32 = (speed_clamped as u32 * self.max_duty) / 100;

        match motor {
            motorselect::right => {
                self.set_direction_right(direction);
                self.pwm_right.set_duty(duty as u8);
            }
            motorselect::left => {
                self.set_direction_left(direction);
                self.pwm_left.set_duty(duty as u8);
            }
        }
    }
    fn set_direction_right(&mut self, dir: direction) {
        match dir {
            direction::forward => {
                self.mr1.set_high();
                self.mr2.set_low();
            }
            direction::backward => {
                self.mr1.set_low();
                self.mr2.set_high();
            }
            direction::coast => {
                self.mr1.set_low();
                self.mr2.set_low();
            }

            direction::brake => {
                self.mr1.set_high();
                self.mr2.set_high();
            }
        }
    }

    fn set_direction_left(&mut self, dir: direction) {
        match dir {
            direction::forward => {
                self.ml1.set_high();
                self.ml2.set_low();
            }
            direction::backward => {
                self.ml1.set_low();
                self.ml2.set_high();
            }
            direction::coast => {
                self.ml1.set_low();
                self.ml2.set_low();
            }

            direction::brake => {
                self.ml1.set_high();
                self.ml2.set_high();
            }
        }
    }
    fn update_frequency(
        &mut self,
        config: &motorconfig, // corrected: accept full config struct
        timer0: &mut timer::timer<'a, lowspeed>,
        timer1: &mut timer::timer<'a, lowspeed>,
    ) {
        // extract frequency from configuration structure
        let new_freq = rate::from_hz(config.frequency);

        // map duty_resolution to ledc duty enum
        let duty_mode = match config.duty_resolution {
            1 => timer::config::duty::duty1bit,
            2 => timer::config::duty::duty2bit,
            3 => timer::config::duty::duty3bit,
            4 => timer::config::duty::duty4bit,
            5 => timer::config::duty::duty5bit,
            6 => timer::config::duty::duty6bit,
            7 => timer::config::duty::duty7bit,
            8 => timer::config::duty::duty8bit,
            9 => timer::config::duty::duty9bit,
            10 => timer::config::duty::duty10bit,
            11 => timer::config::duty::duty11bit,
            12 => timer::config::duty::duty12bit,
            13 => timer::config::duty::duty13bit,
            14 => timer::config::duty::duty14bit,
            _ => timer::config::duty::duty10bit, // default fallback
        };

        // reconfigure timer 0 with new parameters
        timer0
            .configure(timer::config::config {
                duty: duty_mode,
                clock_source: timer::lsclocksource::apbclk,
                frequency: new_freq,
            })
            .ok();

        // reconfigure timer 1 with new parameters
        timer1
            .configure(timer::config::config {
                duty: duty_mode,
                clock_source: timer::lsclocksource::apbclk,
                frequency: new_freq,
            })
            .ok();

        // update internal quantization limit
        self.max_duty = (1u32 << config.duty_resolution) - 1;

        info!(
            "pwm_carrier_reconfigured: f={} hz, n={} bit, max_duty={}",
            new_freq, config.duty_resolution, self.max_duty
        );
    }

    fn stop(&mut self) {
        self.set_motor(motorselect::right, 0, direction::coast);
        self.set_motor(motorselect::left, 0, direction::coast);
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::config::default().with_cpu_clock(cpuclock::max());
    let peripherals = esp_hal::init(config);

    info!("subsystem_initialization_complete");

    let outconfig = outputconfig::default();
    let inconfig = inputconfig::default();

    let mut gpio9 = output::new(peripherals.gpio9, level::high, outconfig);
    let mut gpio10 = output::new(peripherals.gpio10, level::low, outconfig);

    let mut gpio2 = output::new(peripherals.gpio2, level::high, outconfig);
    let mut gpio3 = output::new(peripherals.gpio3, level::low, outconfig);

    let _gpio7 = input::new(peripherals.gpio7, inconfig);
    let _gpio8 = input::new(peripherals.gpio8, inconfig);

    let mut delay = delay::new();

    info!("motor_control_active");
    info!("commencing_operational_loop");

    loop {
        info!("turning right");
    }
}
