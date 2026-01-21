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
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    main,
    time::Rate,
};
use esp_println as _;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        info!("FAULT: UNRECOVERABLE_EXCEPTION_DETECTED");
        let delay = Delay::new();
        delay.delay_millis(1500);
    }
}

#[derive(Clone, Copy)]
enum MotorDirection {
    Clockwise,
    CounterClockwise,
    Brake,
}

#[derive(Clone, Copy)]
enum VehicleMotion {
    Forward,
    Backward,
    Right,
    Left,
    SpinCW,
    SpinCCW,
    Stop,
}

struct MotorController<'a> {
    dir: Output<'a>,
    pwm: channel::Channel<'a, LowSpeed>,
}

impl<'a> MotorController<'a> {
    fn new(dir: Output<'a>, pwm: channel::Channel<'a, LowSpeed>) -> Self {
        Self { dir, pwm }
    }

    fn set_motion(&mut self, direction: MotorDirection, speed: u16, fade_ms: u16) {
        let speed_clamped = speed.min(100) as u8;

        match direction {
            MotorDirection::Clockwise => {
                self.dir.set_low();
                self.pwm.start_duty_fade(0, speed as u8, fade_ms);
            }
            MotorDirection::CounterClockwise => {
                self.dir.set_high();
                loop {
                    self.pwm.set_duty(0);
                    break;
                }
            }
            MotorDirection::Brake => {
                self.dir.set_low();
                loop {
                    self.pwm.set_duty(0);
                    break;
                }
            }
        }
    }
}

struct DifferentialDrive<'a> {
    motor_left: MotorController<'a>,
    motor_right: MotorController<'a>,
}

impl<'a> DifferentialDrive<'a> {
    fn new(motor_left: MotorController<'a>, motor_right: MotorController<'a>) -> Self {
        Self {
            motor_left,
            motor_right,
        }
    }

    fn execute(&mut self, motion: VehicleMotion, speed: u16, fade_ms: u16) {
        let speed_reduced = (speed / 3).min(100);
        let speed_reduced1 = speed.min(100);
        match motion {
            VehicleMotion::Forward => {
                self.motor_left
                    .set_motion(MotorDirection::Clockwise, speed_reduced1, fade_ms);
                self.motor_right.set_motion(
                    MotorDirection::CounterClockwise,
                    speed_reduced1,
                    fade_ms,
                );
            }
            VehicleMotion::Backward => {
                self.motor_left
                    .set_motion(MotorDirection::CounterClockwise, speed, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::Clockwise, speed, fade_ms);
            }
            VehicleMotion::Right => {
                self.motor_left
                    .set_motion(MotorDirection::Brake, 0, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::CounterClockwise, speed, fade_ms);
            }
            VehicleMotion::Left => {
                self.motor_right
                    .set_motion(MotorDirection::Brake, 0, fade_ms);
                self.motor_left
                    .set_motion(MotorDirection::Clockwise, speed, fade_ms);
            }
            VehicleMotion::SpinCCW => {
                self.motor_left
                    .set_motion(MotorDirection::Clockwise, speed, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::Clockwise, speed, fade_ms);
            }
            VehicleMotion::SpinCW => {
                self.motor_left
                    .set_motion(MotorDirection::CounterClockwise, speed, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::CounterClockwise, speed, fade_ms);
            }
            VehicleMotion::Stop => {
                self.motor_left
                    .set_motion(MotorDirection::Brake, 0, fade_ms);
                self.motor_right
                    .set_motion(MotorDirection::Brake, 0, fade_ms);
            }
        }
    }
}

//sensors
struct Sensor<'u> {
    right: Input<'u>,
    center: Input<'u>,
    left: Input<'u>,
}

impl<'u> Sensor<'u> {
    fn read(&mut self, right: bool, center: bool, left: bool) -> (bool, bool, bool) {
        (
            self.right.is_low(),
            self.center.is_low(),
            self.left.is_low(),
        )
    }
}

// maze
const ROWS: usize = 5;
const COLUMNS: usize = 5;
const QUEUE_SIZE_MAX: usize = ROWS * COLUMNS;
const GOAL: (usize, usize) = (2, 2);
const START: (usize, usize) = (0, 4);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Cell {
    distance: u8,
    walls: u8,
}

impl Cell {
    const fn new(distance: u8, walls: u8) -> Self {
        Self {
            distance: 255,
            walls: 0,
        }
    }
}
struct Maze {
    cells: [[Cell; ROWS]; ROWS],
    robot_x: usize,
    robot_y: usize,
    robot_heading: Heading,
    goal_x: usize,
    goal_y: usize,
}

impl Maze {
    fn new(
        cells: [[Cell; ROWS]; ROWS],
        robot_x: usize,
        robot_y: usize,
        robot_heading: Heading,
        goal_x: usize,
        goal_y: usize,
    ) -> Self {
        Self {
            cells: [[Cell::new(255, 0); COLUMNS]; ROWS],
            robot_x: START.0,
            robot_y: START.1,
            robot_heading: Heading::North,
            goal_x: GOAL.0,
            goal_y: GOAL.1,
        }
    }
    fn _flood_fill(&mut self) {
        for row in self.cells.iter_mut() {
            for cell in row.iter_mut() {
                cell.distance = 255;
            }
        }
        self.cells[self.goal_y][self.goal_x].distance = 0;
        let mut queue = [(0usize, 0usize); QUEUE_SIZE_MAX];
        let mut queue_start = 0;
        let mut queue_end = 0;

        queue[queue_end] = (self.goal_x, self.goal_y);
        queue_end += 1;

        const DIR: [(isize, isize); 4] = [(0, -1), (1, 0), (0, 1), (-1, 0)];
        const WALL_DIRS: [u8; 4] = [
            WallState::NORTH,
            WallState::EAST,
            WallState::SOUTH,
            WallState::WEST,
        ];

        while queue_start < queue_end {
            let (x, y) = queue[queue_start];
            queue_start += 1;

            let current_dist = self.cells[y][x].distance;
            for i in 0..4 {
                let (dx, dy) = DIR[i];
                let nx = x as isize + dx;
                let ny = y as isize + dy;

                //checking for bounds
                if nx >= 0 && nx < COLUMNS as isize && ny >= 0 && ny < ROWS as isize {
                    let nx = nx as usize;
                    let ny = ny as usize;

                    if self.cells[y][x].walls & WALL_DIRS[i] == 0 {
                        if self.cells[ny][nx].distance > current_dist + 1 {
                            self.cells[ny][nx].distance = current_dist + 1;
                            queue[queue_end] = (nx, ny);
                            queue_end += 1;
                        }
                    }
                }
            }
        }
    }
    fn update_walls(&mut self, front: bool, left: bool, right: bool) {
        let x = self.robot_x;
        let y = self.robot_y;
        match self.robot_heading {
            Heading::North => {
                if front {
                    self.cells[y][x].walls |= WallState::NORTH;
                }

                if right {
                    self.cells[y][x].walls |= WallState::EAST;
                }

                if left {
                    self.cells[y][x].walls |= WallState::WEST;
                }
            }

            Heading::East => {
                if front {
                    self.cells[y][x].walls |= WallState::EAST;
                }
                if right {
                    self.cells[y][x].walls |= WallState::SOUTH;
                }
                if left {
                    self.cells[y][x].walls |= WallState::NORTH;
                }
            }

            Heading::West => {
                if front {
                    self.cells[y][x].walls |= WallState::WEST;
                }
                if right {
                    self.cells[y][x].walls |= WallState::NORTH;
                }
                if left {
                    self.cells[y][x].walls |= WallState::SOUTH;
                }
            }
            Heading::South => {
                if front {
                    self.cells[y][x].walls |= WallState::SOUTH;
                }
                if right {
                    self.cells[y][x].walls |= WallState::WEST;
                }

                if left {
                    self.cells[y][x].walls |= WallState::EAST;
                }
            }
        }
    }

    fn resolve_policy_step(&self) -> Option<(f32, f32)> {
        let x = self.robot_x;
        let y = self.robot_y;
        if x == self.goal_x && y == self.goal_y {
            return Some((0.0, 0.0)); //need to add spinning here. spin to win
        }
        let current_dist = self.cells[y][x].distance;
        let (front_dist, left_dist, right_dist, back_dist) = match self.robot_heading {
            Heading::North => {
                let front = if y > 0 && (self.cells[y][x].walls & WallState::NORTH == 0) {
                    self.cells[y - 1][x].distance
                } else {
                    255
                };
                let left = if x > 0 && (self.cells[y][x].walls & WallState::WEST == 0) {
                    self.cells[y][x - 1].distance
                } else {
                    255
                };
                let right = if x < COLUMNS - 1 && (self.cells[y][x].walls & WallState::EAST == 0) {
                    self.cells[y][x + 1].distance
                } else {
                    255
                };
                let back = if y < ROWS - 1 && (self.cells[y][x].walls & WallState::SOUTH == 0) {
                    self.cells[y + 1][x].distance
                } else {
                    255
                };
                (front, left, right, back)
            }
            Heading::East => {
                let front = if x < COLUMNS - 1 && (self.cells[y][x].walls & WallState::EAST == 0) {
                    self.cells[y][x + 1].distance
                } else {
                    255
                };
                let left = if y > 0 && (self.cells[y][x].walls & WallState::NORTH == 0) {
                    self.cells[y - 1][x].distance
                } else {
                    255
                };
                let right = if y < ROWS - 1 && (self.cells[y][x].walls & WallState::SOUTH == 0) {
                    self.cells[y + 1][x].distance
                } else {
                    255
                };
                let back = if x > 0 && (self.cells[y][x].walls & WallState::WEST == 0) {
                    self.cells[y][x - 1].distance
                } else {
                    255
                };
                (front, left, right, back)
            }
            Heading::South => {
                let front = if y < ROWS - 1 && (self.cells[y][x].walls & WallState::SOUTH == 0) {
                    self.cells[y + 1][x].distance
                } else {
                    255
                };
                let left = if x < COLUMNS - 1 && (self.cells[y][x].walls & WallState::EAST == 0) {
                    self.cells[y][x + 1].distance
                } else {
                    255
                };
                let right = if x > 0 && (self.cells[y][x].walls & WallState::WEST == 0) {
                    self.cells[y][x - 1].distance
                } else {
                    255
                };
                let back = if y > 0 && (self.cells[y][x].walls & WallState::NORTH == 0) {
                    self.cells[y - 1][x].distance
                } else {
                    255
                };
                (front, left, right, back)
            }
            Heading::West => {
                let front = if x > 0 && (self.cells[y][x].walls & WallState::WEST == 0) {
                    self.cells[y][x - 1].distance
                } else {
                    255
                };
                let left = if y < ROWS - 1 && (self.cells[y][x].walls & WallState::SOUTH == 0) {
                    self.cells[y + 1][x].distance
                } else {
                    255
                };
                let right = if y > 0 && (self.cells[y][x].walls & WallState::NORTH == 0) {
                    self.cells[y - 1][x].distance
                } else {
                    255
                };
                let back = if x < COLUMNS - 1 && (self.cells[y][x].walls & WallState::EAST == 0) {
                    self.cells[y][x + 1].distance
                } else {
                    255
                };
                (front, left, right, back)
            }
        };
        if front_dist < current_dist {
            Some((0.5, 0.5)) // Forward
        } else if left_dist < current_dist {
            Some((0.3, 0.5)) // Turn left
        } else if right_dist < current_dist {
            Some((0.5, 0.3)) // Turn right
        } else if back_dist < current_dist {
            Some((-0.5, 0.5)) // 180 degree turn
        } else {
            None
        }
    }

    fn update_position(&mut self, left_speed: f32, right_speed: f32) {
        if left_speed > 0.0 && right_speed > 0.0 {
            // Moving forward
            match self.robot_heading {
                Heading::North => {
                    if self.robot_y > 0 {
                        self.robot_y -= 1;
                    }
                }
                Heading::East => {
                    if self.robot_x < COLUMNS - 1 {
                        self.robot_x += 1;
                    }
                }
                Heading::South => {
                    if self.robot_y < ROWS - 1 {
                        self.robot_y += 1;
                    }
                }
                Heading::West => {
                    if self.robot_x > 0 {
                        self.robot_x -= 1;
                    }
                }
            }
        } else if left_speed < right_speed {
            // Turning right
            self.robot_heading = match self.robot_heading {
                Heading::North => Heading::East,
                Heading::East => Heading::South,
                Heading::South => Heading::West,
                Heading::West => Heading::North,
            };
        } else if left_speed > right_speed {
            // Turning left
            self.robot_heading = match self.robot_heading {
                Heading::North => Heading::West,
                Heading::East => Heading::North,
                Heading::South => Heading::East,
                Heading::West => Heading::South,
            };
        }
    }
    fn is_locked() {
        todo!()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Heading {
    North = 0,
    East = 1,
    South = 2,
    West = 3,
}

struct WallState;

impl WallState {
    const NORTH: u8 = 0b0001;
    const EAST: u8 = 0b0010;
    const SOUTH: u8 = 0b0100;
    const WEST: u8 = 0b1000;
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let outconfig = OutputConfig::default();
    let inconfig = InputConfig::default();

    // H-BRIDGE INTERFACE ALLOCATION
    let r_dir = Output::new(peripherals.GPIO7, Level::Low, outconfig);
    let r_pwm = peripherals.GPIO6;
    let l_dir = Output::new(peripherals.GPIO2, Level::Low, outconfig);
    let l_pwm = peripherals.GPIO3;

    let mut encoder1 = Input::new(peripherals.GPIO8, inconfig);
    let mut encoder0 = Input::new(peripherals.GPIO4, inconfig);
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // RIGHT MOTOR PWM TIMER (10-BIT RESOLUTION)
    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0.configure(timer::config::Config {
        duty: timer::config::Duty::Duty10Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(20),
    });

    let mut channel0 = ledc.channel(channel::Number::Channel0, r_pwm);
    channel0.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    });

    // LEFT MOTOR PWM TIMER (10-BIT RESOLUTION, SHARED TIMER)
    let mut channel1 = ledc.channel(channel::Number::Channel1, l_pwm);
    channel1.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    });

    //let mut sensor_right = Input::new(peripherals.GPIO4, inconfig);
    //let mut sensor_center = Input::new(peripherals.GPIO5, inconfig);
    ////let mut sensor_left = Input::new(peripherals.GPIO6, inconfig);
    //let mut sensors: Sensor = Sensor {
    //    right: sensor_right,
    //    center: sensor_center,
    //    //left: sensor_left,
    //};

    //let _gpio7 = Input::new(peripherals.GPIO7, inconfig);
    //let _gpio8 = Input::new(peripherals.GPIO8, inconfig);

    let motor_right = MotorController::new(r_dir, channel0);
    let motor_left = MotorController::new(l_dir, channel1);

    let mut drive = DifferentialDrive::new(motor_left, motor_right);

    let mut delay = Delay::new();

    info!("DIFFERENTIAL_DRIVE_INITIALIZED: ENTERING_OPERATIONAL_LOOP");
    let mut prev = encoder0.is_high();
    let mut edges = 0;
    const CELL_TIME_MS: u64 = 1500;
    info!("DIFFERENTIAL_DRIVE_INITIALIZED: ENTERING_OPERATIONAL_LOOP");

    loop {
        info!("Moving forward one cell");

        let mut prev = encoder0.is_high();
        let mut edges = 0;

        drive.execute(VehicleMotion::Forward, 100, 300);

        // Simple blocking loop for 1.5 seconds
        for _ in 0..16 {
            // 15 iterations
            delay.delay_millis(100); // 100 ms per iteration → 15 * 100 = 1500 ms
            let now = encoder0.is_high();
            if now != prev {
                edges += 1;
                prev = now;

                // MANUAL: print 0 or 1 at each edge
                if now {
                    info!("1");
                } else {
                    info!("0");
                }
            }
        }

        drive.execute(VehicleMotion::Stop, 100, 300);
        info!("Cell traversal complete, total edges counted: {}", edges);

        delay.delay_millis(500); // pause before next cell
    }
}
