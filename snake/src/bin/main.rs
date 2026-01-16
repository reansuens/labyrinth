#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is unsafe with esp_hal types holding buffers."
)]

use defmt::info;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pin},
    ledc::{
        channel::{self, ChannelIFace, Error},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    main,
    time::Rate,
    timer::PeriodicTimer,
};
use esp_println as _;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        info!("FAULT: UNRECOVERABLE_EXCEPTION_DETECTED");
    }
}

#[derive(Clone, Copy)]
enum MotorSelect {
    Right,
    Left,
}

#[derive(Clone, Copy)]
enum Direction {
    Forward,
    Backward,
    Brake,
    Coast,
}

#[derive(Debug, Clone)]
struct MotorConfig {
    frequency: u32,
    duty_resolution: u8,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            frequency: 15000,
            duty_resolution: 10,
        }
    }
}

struct MotorController<'a> {
    r_dir: Output<'a>,
    l_dir: Output<'a>,
    pwm_right: channel::Channel<'a, LowSpeed>,
    pwm_left: channel::Channel<'a, LowSpeed>,
    max_duty: u32,
}

impl<'a> MotorController<'a> {
    fn new(
        r_dir: Output<'a>,
        l_dir: Output<'a>,
        pwm_right: channel::Channel<'a, LowSpeed>,
        pwm_left: channel::Channel<'a, LowSpeed>,
        config: &MotorConfig,
    ) -> Self {
        let max_duty = (1u32 << config.duty_resolution) - 1;

        Self {
            r_dir,
            l_dir,
            pwm_right,
            pwm_left,
            max_duty,
        }
    }

    fn set_motor(&mut self, motor: MotorSelect, speed: u8, direction: Direction) {
        let speed = speed.min(100);
        let duty = (speed as u32 * self.max_duty) / 100;

        match motor {
            MotorSelect::Right => {
                match direction {
                    Direction::Forward => self.r_dir.set_high(),
                    Direction::Backward => self.r_dir.set_low(),
                    _ => self.pwm_right.set_duty(0),
                }
                self.pwm_right.set_duty(duty as u8);
            }

            MotorSelect::Left => {
                match direction {
                    Direction::Forward => self.l_dir.set_high(),
                    Direction::Backward => self.l_dir.set_low(),
                    _ => self.pwm_left.set_duty(0),
                }
                self.pwm_left.set_duty(duty as u8);
            }
        }
    }

    fn update_frequency(
        &mut self,
        config: &MotorConfig,
        timer0: &mut timer::Timer<'a, LowSpeed>,
        timer1: &mut timer::Timer<'a, LowSpeed>,
    ) {
        let new_freq = Rate::from_hz(config.frequency);

        let duty_mode = match config.duty_resolution {
            1 => timer::config::Duty::Duty1Bit,
            2 => timer::config::Duty::Duty2Bit,
            3 => timer::config::Duty::Duty3Bit,
            4 => timer::config::Duty::Duty4Bit,
            5 => timer::config::Duty::Duty5Bit,
            6 => timer::config::Duty::Duty6Bit,
            7 => timer::config::Duty::Duty7Bit,
            8 => timer::config::Duty::Duty8Bit,
            9 => timer::config::Duty::Duty9Bit,
            10 => timer::config::Duty::Duty10Bit,
            11 => timer::config::Duty::Duty11Bit,
            12 => timer::config::Duty::Duty12Bit,
            13 => timer::config::Duty::Duty13Bit,
            14 => timer::config::Duty::Duty14Bit,
            _ => timer::config::Duty::Duty10Bit,
        };

        timer0
            .configure(timer::config::Config {
                duty: duty_mode,
                clock_source: timer::LSClockSource::APBClk,
                frequency: new_freq,
            })
            .ok();

        timer1
            .configure(timer::config::Config {
                duty: duty_mode,
                clock_source: timer::LSClockSource::APBClk,
                frequency: new_freq,
            })
            .ok();

        self.max_duty = (1u32 << config.duty_resolution) - 1;

        info!(
            "PWM_CARRIER_RECONFIGURED: f={} Hz, N={} bit, max_duty={}",
            new_freq, config.duty_resolution, self.max_duty
        );
    }

    fn stop(&mut self) {
        self.set_motor(MotorSelect::Right, 0, Direction::Coast);
        self.set_motor(MotorSelect::Left, 0, Direction::Coast);
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let outconfig = OutputConfig::default();
    let inconfig = InputConfig::default();

    // H-BRIDGE DIRECTION CONTROL PINS (STATIC OUTPUT)
    let r_dir = Output::new(peripherals.GPIO9, Level::Low, outconfig); // DIR
    let r_pwm = peripherals.GPIO10; // PWM
    let l_dir = Output::new(peripherals.GPIO2, Level::Low, outconfig); // DIR
    let l_pwm = peripherals.GPIO3; // PWM

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0.configure(timer::config::Config {
        duty: timer::config::Duty::Duty5Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(15),
    });

    let mut channel0 = ledc.channel(channel::Number::Channel0, r_pwm);
    channel0.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 10,
        drive_mode: DriveMode::PushPull,
    });

    // left
    let mut lstimer1 = ledc.timer::<LowSpeed>(timer::Number::Timer1);
    lstimer0.configure(timer::config::Config {
        duty: timer::config::Duty::Duty5Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(15),
    });

    let mut channel1 = ledc.channel(channel::Number::Channel1, l_pwm);
    channel0.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 10,
        drive_mode: DriveMode::PushPull,
    });
    // INPUT SENSOR ACQUISITION
    let _gpio7 = Input::new(peripherals.GPIO7, inconfig);
    let _gpio8 = Input::new(peripherals.GPIO8, inconfig);

    // MOTOR CONTROLLER INSTANTIATION (CORRECTED OWNERSHIP)
    let motor_cfg = MotorConfig::default();

    let mut motor = MotorController::new(r_dir, l_dir, channel0, channel1, &motor_cfg);
    let mut delay = Delay::new();

    info!("MOTOR_CONTROLLER_INITIALIZED: ENTERING_MAIN_CONTROL_LOOP");

    // MAIN CONTROL SEQUENCE
    loop {
        // FORWARD VECTOR EXECUTION
        motor_controller.set_motor(MotorSelect::Right, 80, Direction::Forward);
        motor_controller.set_motor(MotorSelect::Left, 80, Direction::Forward);
        delay.delay_millis(2000);

        // REVERSE VECTOR EXECUTION
        motor_controller.set_motor(MotorSelect::Right, 60, Direction::Backward);
        motor_controller.set_motor(MotorSelect::Left, 60, Direction::Backward);
        delay.delay_millis(2000);

        // BRAKE APPLICATION
        motor_controller.set_motor(MotorSelect::Right, 0, Direction::Brake);
        motor_controller.set_motor(MotorSelect::Left, 0, Direction::Brake);
        delay.delay_millis(1000);

        // COAST MODE (ZERO TORQUE)
        motor_controller.stop();
        delay.delay_millis(1000);
    }
}
