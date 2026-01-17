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
enum Motion {
    Forward,
    Backward,
    Right,
    Left,
    SpinCW,
    SpinCCW,
    Stop,
}

struct MotorController<'a> {
    r_dir: Output<'a>,
    l_dir: Output<'a>,
    pwm_right: channel::Channel<'a, LowSpeed>,
    pwm_left: channel::Channel<'a, LowSpeed>,
}

impl<'a> MotorController<'a> {
    fn new(
        r_dir: Output<'a>,
        l_dir: Output<'a>,
        pwm_right: channel::Channel<'a, LowSpeed>,
        pwm_left: channel::Channel<'a, LowSpeed>,
    ) -> Self {
        Self {
            r_dir,
            l_dir,
            pwm_right,
            pwm_left,
        }
    }
    fn execute_motion(&mut self, motion: Motion, speed: u16, fade_ms: u16) {
        let speed_clamped = speed.min(100);
        let halved: u8 = speed as u8 / 3;
        match motion {
            Motion::Forward => {
                info!("forward motion");
                self.r_dir.set_high();
                self.pwm_right.start_duty_fade(0, speed as u8, fade_ms);
                self.l_dir.set_low();
                self.pwm_left.start_duty_fade(0, speed as u8, fade_ms);
            }

            Motion::Backward => {
                info!("Backward motion");
                self.r_dir.set_low();
                self.pwm_right.start_duty_fade(0, speed as u8, fade_ms);
                self.l_dir.set_high();
                self.pwm_left.start_duty_fade(0, speed as u8, fade_ms);
            }

            Motion::Left => {
                info!("LEFT TURN");
                self.r_dir.set_high();
                self.pwm_right.start_duty_fade(0, speed as u8, fade_ms);
                self.l_dir.set_low();
                self.pwm_left.start_duty_fade(0, halved, fade_ms);
            }

            Motion::Right => {
                info!("RIGHT TURN");
                self.r_dir.set_low();
                self.pwm_right.start_duty_fade(0, halved, fade_ms);
                self.l_dir.set_high();
                self.pwm_left.start_duty_fade(0, speed as u8, fade_ms);
            }
            Motion::SpinCW => {
                info!("CLOCKWISE SPIN");
                self.r_dir.set_low();
                self.pwm_right.start_duty_fade(0, speed as u8, fade_ms);
                self.r_dir.set_low();
                self.pwm_left.start_duty_fade(0, speed as u8, fade_ms);
            }

            Motion::SpinCCW => {
                info!("COUNTER CLOCKWISE SPIN");
                self.r_dir.set_high();
                self.pwm_right.start_duty_fade(0, speed as u8, fade_ms);
                self.r_dir.set_high();
                self.pwm_left.start_duty_fade(0, speed as u8, fade_ms);
            }
            Motion::Stop => {
                info!("COMPLETE STOP");
                self.r_dir.set_low();
                self.l_dir.set_low();

                self.pwm_right.start_duty_fade(0, 0, fade_ms);
                self.pwm_left.start_duty_fade(0, 0, fade_ms);
            }
        }
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
    let mut r_dir = Output::new(peripherals.GPIO9, Level::Low, outconfig); // DIR
    let r_pwm = peripherals.GPIO10; // PWM

    let mut l_dir = Output::new(peripherals.GPIO2, Level::Low, outconfig); // DIR
    let l_pwm = peripherals.GPIO3; // PWM

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0.configure(timer::config::Config {
        duty: timer::config::Duty::Duty10Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(1),
    });
    let mut channel0 = ledc.channel(channel::Number::Channel0, r_pwm);
    channel0.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 15,
        drive_mode: DriveMode::PushPull,
    });

    // left
    let mut lstimer1 = ledc.timer::<LowSpeed>(timer::Number::Timer1);

    lstimer1.configure(timer::config::Config {
        duty: timer::config::Duty::Duty5Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(1),
    });
    let mut channel1 = ledc.channel(channel::Number::Channel1, l_pwm);
    channel1.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 15,
        drive_mode: DriveMode::PushPull,
    });
    // INPUT SENSOR ACQUISITION
    let _gpio7 = Input::new(peripherals.GPIO7, inconfig);
    let _gpio8 = Input::new(peripherals.GPIO8, inconfig);

    // MOTOR CONTROLLER INSTANTIATION (CORRECTED OWNERSHIP)

    //let mut motor = MotorController::new(r_dir, l_dir, channel0, channel1, &motor_cfg);
    let mut delay = Delay::new();

    info!("MOTOR_CONTROLLER_INITIALIZED: ENTERING_MAIN_CONTROL_LOOP");

    let mut Motor = MotorController::new(r_dir, l_dir, channel0, channel1);
    loop {
        Motor.execute_motion(Motion::Forward, 100, 100);
        delay.delay_millis(1000);
        Motor.execute_motion(Motion::Backward, 100, 100);
        delay.delay_millis(1000);
        // FORWARD VECTOR: BOTH MOTORS
    }
}
