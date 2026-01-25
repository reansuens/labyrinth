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
    left: Input<'u>,
    center: Input<'u>,
    right: Input<'u>,
}

enum IsWall {
    Left,
    Front,
    Right,
}

impl<'u> Sensor<'u> {
    fn read(
        &mut self,
        walled: IsWall,
        sleft: bool,
        scenter: bool,
        sright: bool,
    ) -> (bool, bool, bool) {
        match walled {
            IsWall::Left => (true, scenter, sleft),
            IsWall::Front => (sright, true, sleft),
            IsWall::Right => (sright, scenter, true),
        }
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

struct Encoders<'e> {
    zero_left: Input<'e>,
    zero_right: Input<'e>,
}

impl<'e> Encoders<'e> {
    fn new(&mut self, zero_left: Input<'e>, zero_right: Input<'e>) -> Self {
        Self {
            zero_left,
            zero_right,
        }
    }
    fn forward_one(&self, drive: &mut DifferentialDrive) {
        let mut delay = Delay::new();
        let mut prev0 = self.zero_left.is_high();
        let mut prev1 = self.zero_right.is_high();
        let mut edge0 = 0;
        let mut edge1 = 0;
        drive.execute(VehicleMotion::Forward, 100, 300);
        for _ in 0..14 {
            delay.delay_millis(100);
            let now0 = self.zero_left.is_high();
            let now1 = self.zero_right.is_high();
            if (now0 != prev0) && (now1 != prev1) {
                edge0 += 1;
                prev0 = now0;
                edge1 += 1;
                prev1 = now1;

                if (edge0 == 8) | (edge1 == 8) {
                    drive.execute(VehicleMotion::Stop, 0, 0);
                    delay.delay_millis(150);
                }
            }
        }
    }
}
impl Cell {
    const fn new(distance: u8, walls: u8) -> Self {
        Self { distance, walls }
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
            cells,
            robot_x,
            robot_y,
            robot_heading,
            goal_x,
            goal_y,
        }
    }
    fn resolve_forward_scan(
        &mut self,
        encoders: &mut Encoders,
        sensor: &mut Sensor,
        drive: &mut DifferentialDrive,
    ) -> Option<(bool, bool, bool)> {
        // PHASE 1: Execute forward motion primitive
        encoders.forward_one(drive);

        // PHASE 2: Update robot pose based on current heading
        match self.robot_heading {
            Heading::North => {
                if self.robot_x > 0 {
                    self.robot_x -= 1;
                } else {
                    return None; // Boundary collision detected
                }
            }
            Heading::East => {
                if self.robot_y < COLUMNS - 1 {
                    self.robot_y += 1;
                } else {
                    return None;
                }
            }
            Heading::South => {
                if self.robot_x < ROWS - 1 {
                    self.robot_x += 1;
                } else {
                    return None;
                }
            }
            Heading::West => {
                if self.robot_y > 0 {
                    self.robot_y -= 1;
                } else {
                    return None;
                }
            }
        }

        // PHASE 3: Acquire sensor data (active-low correction)
        let left_raw = sensor.left.is_high();
        let center_raw = sensor.center.is_high();
        let right_raw = sensor.right.is_high();

        // Invert for active-low sensors (LOW = wall present)
        let wall_left = !left_raw;
        let wall_front = !center_raw;
        let wall_right = !right_raw;

        // PHASE 4: Update wall topology for current cell
        self.wall_state(sensor, wall_left, wall_front, wall_right);

        // PHASE 5: Conditional double-step if corridor detected
        if !wall_left && !wall_front && !wall_right {
            // Open corridor — execute second cell traversal
            encoders.forward_one(drive);

            // Update pose again
            match self.robot_heading {
                Heading::North => {
                    if self.robot_x > 0 {
                        self.robot_x -= 1;
                    }
                }
                Heading::East => {
                    if self.robot_y < COLUMNS - 1 {
                        self.robot_y += 1;
                    }
                }
                Heading::South => {
                    if self.robot_x < ROWS - 1 {
                        self.robot_x += 1;
                    }
                }
                Heading::West => {
                    if self.robot_y > 0 {
                        self.robot_y -= 1;
                    }
                }
            }
        }

        // PHASE 6: Return processed sensor state
        Some((wall_left, wall_front, wall_right))
    }
    fn flood_fill(&mut self) {
        // Initialize distance field to maximum
        for row in 0..ROWS {
            for col in 0..COLUMNS {
                self.cells[row][col].distance = u8::MAX;
            }
        }

        // Seed goal cell
        self.cells[self.goal_x][self.goal_y].distance = 0;

        // BFS queue allocation (circular buffer preferred for determinism)
        let mut queue: [(usize, usize); QUEUE_SIZE_MAX] = [(0, 0); QUEUE_SIZE_MAX];
        let mut head: usize = 0;
        let mut tail: usize = 0;

        queue[tail] = (self.goal_x, self.goal_y);
        tail = (tail + 1) % QUEUE_SIZE_MAX;

        while head != tail {
            let (x, y) = queue[head];
            head = (head + 1) % QUEUE_SIZE_MAX;

            let current_dist = self.cells[x][y].distance;

            // Check all cardinal neighbors (N, E, S, W)
            let neighbors = [
                (x.wrapping_sub(1), y, WallState::NORTH), // North
                (x, y + 1, WallState::EAST),              // East
                (x + 1, y, WallState::SOUTH),             // South
                (x, y.wrapping_sub(1), WallState::WEST),  // West
            ];

            for &(nx, ny, wall_bit) in &neighbors {
                if nx < ROWS && ny < COLUMNS {
                    // Check if wall exists in current cell
                    if (self.cells[x][y].walls & wall_bit) == 0 {
                        if self.cells[nx][ny].distance > current_dist + 1 {
                            self.cells[nx][ny].distance = current_dist + 1;
                            queue[tail] = (nx, ny);
                            tail = (tail + 1) % QUEUE_SIZE_MAX;
                        }
                    }
                }
            }
        }
    }
    fn wall_state(&mut self, sensor: &mut Sensor, left: bool, center: bool, right: bool) {
        let (wall_left, wall_front, wall_right) = sensor.read(
            IsWall::Front, // Placeholder — adjust based on physical sensor orientation
            left,
            center,
            right,
        );

        // Active-low correction: invert sensor logic
        //
        let wall_left = !wall_left;
        let wall_front = !wall_front;
        let wall_right = !wall_right;

        // Map relative walls to absolute directions
        let current_walls = &mut self.cells[self.robot_x][self.robot_y].walls;

        match self.robot_heading {
            Heading::North => {
                if wall_left {
                    *current_walls |= WallState::WEST;
                }
                if wall_front {
                    *current_walls |= WallState::NORTH;
                }
                if wall_right {
                    *current_walls |= WallState::EAST;
                }
            }
            Heading::East => {
                if wall_left {
                    *current_walls |= WallState::NORTH;
                }
                if wall_front {
                    *current_walls |= WallState::EAST;
                }
                if wall_right {
                    *current_walls |= WallState::SOUTH;
                }
            }
            Heading::South => {
                if wall_left {
                    *current_walls |= WallState::EAST;
                }
                if wall_front {
                    *current_walls |= WallState::SOUTH;
                }
                if wall_right {
                    *current_walls |= WallState::WEST;
                }
            }
            Heading::West => {
                if wall_left {
                    *current_walls |= WallState::SOUTH;
                }
                if wall_front {
                    *current_walls |= WallState::WEST;
                }
                if wall_right {
                    *current_walls |= WallState::NORTH;
                }
            }
        }
    }
    fn resolve_policy_step(&mut self) -> Option<(usize, usize)> {
        let (x, y) = (self.robot_x, self.robot_y);
        let current_dist = self.cells[x][y].distance;

        let mut best_neighbor: Option<(usize, usize)> = None;
        let mut min_distance = current_dist;

        // Evaluate neighbors (N, E, S, W)
        let neighbors = [
            (x.wrapping_sub(1), y, WallState::NORTH),
            (x, y + 1, WallState::EAST),
            (x + 1, y, WallState::SOUTH),
            (x, y.wrapping_sub(1), WallState::WEST),
        ];

        for &(nx, ny, wall_bit) in &neighbors {
            if nx < ROWS && ny < COLUMNS {
                // Check accessibility
                if (self.cells[x][y].walls & wall_bit) == 0 {
                    if self.cells[nx][ny].distance < min_distance {
                        min_distance = self.cells[nx][ny].distance;
                        best_neighbor = Some((nx, ny));
                    }
                }
            }
        }

        best_neighbor
    }

    fn update_robot_pose(&mut self, target_x: usize, target_y: usize) {
        // Determine required heading
        let required_heading = if target_x < self.robot_x {
            Heading::North
        } else if target_x > self.robot_x {
            Heading::South
        } else if target_y > self.robot_y {
            Heading::East
        } else {
            Heading::West
        };

        // Update position
        self.robot_x = target_x;
        self.robot_y = target_y;
        self.robot_heading = required_heading;
    }
}

enum Heading {
    North = 0,
    East = 1,
    South = 2,
    West = 3,
}

struct WallState {
    bits: u8,
}

impl WallState {
    const NORTH: u8 = 0b0001;
    const EAST: u8 = 0b0010;
    const SOUTH: u8 = 0b0100;
    const WEST: u8 = 0b1000;
    fn new(&self) -> Self {
        WallState { bits: 0 }
    }

    fn is_locked() {
        todo!()
    }
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

        delay.delay_millis(500); 
    }
}
