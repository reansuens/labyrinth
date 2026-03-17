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
    //true = wall present
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
const GOAL: (usize, usize) = (1, 2);
const START: (usize, usize) = (2, 3);

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
    fn new(zero_left: Input<'e>, zero_right: Input<'e>) -> Self {
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

const WALL_NORTH: u8 = 0b0001;
const WALL_EAST: u8 = 0b0010;
const WALL_SOUTH: u8 = 0b0100;
const WALL_WEST: u8 = 0b1000;

struct Maze {
    cells: [[Cell; ROWS]; ROWS],
    robot_x: usize,
    robot_y: usize,
    robot_heading: Heading,
    goal_x: usize,
    goal_y: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Heading {
    North = 0,
    East = 1,
    South = 2,
    West = 3,
}
impl Heading {
    fn delta(self, target: Heading) -> i8 {
        let raw_delta = (target as i8 - self as i8 + 4) % 4;
        // Normalize to [-2, 2] for shortest rotation
        if raw_delta > 2 {
            raw_delta - 4
        } else {
            raw_delta
        }
    }
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
        // Check bounds first
        let can_move = match self.robot_heading {
            Heading::North => self.robot_x > 0,
            Heading::East => self.robot_y < COLUMNS - 1,
            Heading::South => self.robot_x < ROWS - 1,
            Heading::West => self.robot_y > 0,
        };
        if !can_move {
            return None;
        }
        // Move physically first
        encoders.forward_one(drive);
        // Update position (safe because we checked bounds)
        match self.robot_heading {
            Heading::North => self.robot_x -= 1,
            Heading::East => self.robot_y += 1,
            Heading::South => self.robot_x += 1,
            Heading::West => self.robot_y -= 1,
        }
        // Read sensors
        let left_raw = sensor.left.is_high();
        let center_raw = sensor.center.is_high();
        let right_raw = sensor.right.is_high();
        let wall_left = !left_raw;
        let wall_center = !center_raw;
        let wall_right = !right_raw;
        self.wall_state_update(wall_left, wall_center, wall_right);
        // If no walls, move forward one more cell
        if !wall_left && !wall_center && !wall_right {
            // Check bounds for second move
            let can_move_again = match self.robot_heading {
                Heading::North => self.robot_x > 0,
                Heading::East => self.robot_y < COLUMNS - 1,
                Heading::South => self.robot_x < ROWS - 1,
                Heading::West => self.robot_y > 0,
            };
            if can_move_again {
                encoders.forward_one(drive);
                match self.robot_heading {
                    Heading::North => self.robot_x -= 1,
                    Heading::East => self.robot_y += 1,
                    Heading::South => self.robot_x += 1,
                    Heading::West => self.robot_y -= 1,
                }
            }
        }
        Some((wall_left, wall_center, wall_right))
    }

    fn wall_state_update(&mut self, wall_left: bool, wall_center: bool, wall_right: bool) {
        let current_walls = &mut self.cells[self.robot_x][self.robot_y].walls;
        match self.robot_heading {
            Heading::North => {
                if wall_left {
                    *current_walls |= WALL_WEST;
                }
                if wall_center {
                    *current_walls |= WALL_NORTH;
                }
                if wall_right {
                    *current_walls |= WALL_EAST;
                }
            }
            Heading::East => {
                if wall_left {
                    *current_walls |= WALL_NORTH;
                }
                if wall_center {
                    *current_walls |= WALL_EAST;
                }
                if wall_right {
                    *current_walls |= WALL_SOUTH;
                }
            }
            Heading::South => {
                if wall_left {
                    *current_walls |= WALL_EAST;
                }
                if wall_center {
                    *current_walls |= WALL_SOUTH;
                }
                if wall_right {
                    *current_walls |= WALL_WEST;
                }
            }
            Heading::West => {
                if wall_left {
                    *current_walls |= WALL_SOUTH;
                }
                if wall_center {
                    *current_walls |= WALL_WEST;
                }
                if wall_right {
                    *current_walls |= WALL_NORTH;
                }
            }
        }
    }

    fn flood_fill(&mut self) {
        for row in 0..ROWS {
            for col in 0..COLUMNS {
                self.cells[row][col].distance = u8::MAX;
            }
        }

        self.cells[self.goal_x][self.goal_y].distance = 0;
        let mut queue: [(usize, usize); QUEUE_SIZE_MAX] = [(0, 0); QUEUE_SIZE_MAX];
        let mut head: usize = 0;
        let mut tail: usize = 0;
        queue[tail] = (self.goal_x, self.goal_y);
        tail = (tail + 1) % QUEUE_SIZE_MAX;

        while head != tail {
            let (x, y) = queue[head];
            head = (head + 1) % QUEUE_SIZE_MAX;
            let current_dist = self.cells[x][y].distance;
            let neighbors = [
                (x.wrapping_sub(1), y, WALL_NORTH),
                (x, y + 1, WALL_EAST),
                (x + 1, y, WALL_SOUTH),
                (x, y.wrapping_sub(1), WALL_WEST),
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

    fn resolve_policy_step(&mut self) -> Option<(usize, usize, i8)> {
        let (x, y) = (self.robot_x, self.robot_y);
        let current_dist = self.cells[x][y].distance;

        let mut best_neighbor: Option<(usize, usize)> = None;
        let mut min_distance = current_dist;

        let neighbors = [
            (x.wrapping_sub(1), y, WALL_NORTH),
            (x, y + 1, WALL_EAST),
            (x + 1, y, WALL_SOUTH),
            (x, y.wrapping_sub(1), WALL_WEST),
        ];

        for &(nx, ny, wall_bit) in &neighbors {
            if nx < ROWS && ny < COLUMNS {
                if (self.cells[x][y].walls & wall_bit) == 0 {
                    if self.cells[nx][ny].distance < min_distance {
                        min_distance = self.cells[nx][ny].distance;
                        best_neighbor = Some((nx, ny));
                    }
                }
            }
        }

        if let Some((target_x, target_y)) = best_neighbor {
            let required_heading = self.compute_required_heading(target_x, target_y);
            let heading_delta = self.robot_heading.delta(required_heading);
            return Some((target_x, target_y, heading_delta));
        }
        None
    }
    fn compute_required_heading(&mut self, target_x: usize, target_y: usize) -> Heading {
        if target_x < self.robot_x {
            Heading::North
        } else if target_x > self.robot_x {
            Heading::South
        } else if target_y > self.robot_y {
            Heading::East
        } else {
            Heading::West
        }
    }

    fn update_robot_pose(&mut self, target_x: usize, target_y: usize) {
        let required_heading = if target_x < self.robot_x {
            Heading::North
        } else if target_x > self.robot_x {
            Heading::South
        } else if target_y < self.robot_y {
            Heading::West
        } else {
            Heading::East
        };
        self.robot_x = target_x;
        self.robot_y = target_y;
        self.robot_heading = required_heading;
    }

    fn execute_rotation(
        &mut self,
        heading_delta: i8,
        drive: &mut DifferentialDrive,
        delay: &mut Delay,
    ) {
        const ROTATION_90_MS: u32 = 780;
        const ROTATION_180_MS: u32 = 1500;

        match heading_delta {
            1 | -3 => {
                drive.execute(VehicleMotion::SpinCW, 100, 300);
                delay.delay_millis(ROTATION_90_MS);
                drive.execute(VehicleMotion::Stop, 0, 0);

                self.robot_heading = match self.robot_heading {
                    Heading::North => Heading::East,
                    Heading::East => Heading::South,
                    Heading::South => Heading::West,
                    Heading::West => Heading::North,
                };
            }

            -1 | 3 => {
                // 90° counter-clockwise
                drive.execute(VehicleMotion::SpinCCW, 100, 300);
                delay.delay_millis(ROTATION_90_MS);
                drive.execute(VehicleMotion::Stop, 0, 0);

                self.robot_heading = match self.robot_heading {
                    Heading::North => Heading::West,
                    Heading::East => Heading::North,
                    Heading::South => Heading::East,
                    Heading::West => Heading::South,
                };
            }
            2 | -2 => {
                drive.execute(VehicleMotion::SpinCW, 100, 300);
                delay.delay_millis(ROTATION_180_MS);
                drive.execute(VehicleMotion::Stop, 0, 0);

                self.robot_heading = match self.robot_heading {
                    Heading::North => Heading::South,
                    Heading::East => Heading::West,
                    Heading::South => Heading::North,
                    Heading::West => Heading::East,
                };
            }

            0 => {}
            _ => {
                info!("FAULT: INVALID HEADING DELTA = {},", heading_delta);
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

    // H-BRIDGE INTERFACE ALLOCATION
    let r_dir = Output::new(peripherals.GPIO7, Level::Low, outconfig);
    let r_pwm = peripherals.GPIO6;
    let l_dir = Output::new(peripherals.GPIO2, Level::Low, outconfig);
    let l_pwm = peripherals.GPIO3;

    let mut encoder1 = Input::new(peripherals.GPIO21, inconfig);
    let mut encoder0 = Input::new(peripherals.GPIO9, inconfig);

    let mut encoders = Encoders::new(encoder1, encoder0);

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

    let mut sensor_right = Input::new(peripherals.GPIO4, inconfig);
    let mut sensor_center = Input::new(peripherals.GPIO5, inconfig);
    let mut sensor_left = Input::new(peripherals.GPIO20, inconfig);
    let mut sensors: Sensor = Sensor {
        right: sensor_right,
        center: sensor_center,
        left: sensor_left,
    };

    let motor_right = MotorController::new(r_dir, channel0);
    let motor_left = MotorController::new(l_dir, channel1);

    let mut drive = DifferentialDrive::new(motor_left, motor_right);

    let mut delay = Delay::new();

    let mut maze = Maze::new(
        [[Cell::new(u8::MAX, 0); ROWS]; ROWS],
        START.0,
        START.1,
        Heading::East,
        GOAL.0,
        GOAL.1,
    );

    delay.delay_millis(2000); // Startup settling time

    info!("DIFFERENTIAL_DRIVE_INITIALIZED: ENTERING_OPERATIONAL_LOOP");

    loop {
        // PHASE 1: SENSOR ACQUISITION & WALL MAPPING
        if let Some((_wl, _wc, _wr)) =
            maze.resolve_forward_scan(&mut encoders, &mut sensors, &mut drive)
        {
            // PHASE 2: RECOMPUTE DISTANCE FIELD
            maze.flood_fill();

            // PHASE 3: GOAL PROXIMITY CHECK
            if (maze.robot_x == maze.goal_x) && (maze.robot_y == maze.goal_y) {
                drive.execute(VehicleMotion::Stop, 0, 0);
                info!(
                    "MISSION_COMPLETE: GOAL_REACHED at ({}, {})",
                    maze.goal_x, maze.goal_y
                );

                // Celebration sequence
                for _ in 0..3 {
                    drive.execute(VehicleMotion::SpinCW, 80, 200);
                    delay.delay_millis(5000);
                    drive.execute(VehicleMotion::Stop, 0, 0);
                    delay.delay_millis(200);
                }

                break; // Exit to terminal state
            }

            // PHASE 4: POLICY GRADIENT EVALUATION
            if let Some((target_x, target_y, heading_delta)) = maze.resolve_policy_step() {
                // PHASE 5: HEADING ALIGNMENT
                if heading_delta != 0 {
                    maze.execute_rotation(heading_delta, &mut drive, &mut delay);
                    delay.delay_millis(200); // Gyro settling time
                }

                // PHASE 6: TRANSLATION (handled by next resolve_forward_scan)
                // Note: Motion occurs at start of next loop iteration
            } else {
                // No valid path found
                drive.execute(VehicleMotion::Stop, 0, 0);
                info!("FAULT: NO_VALID_POLICY - Robot trapped or distance field corrupted");
                break;
            }
        } else {
            // Boundary collision detected
            drive.execute(VehicleMotion::Stop, 0, 0);
            info!(
                "FAULT: BOUNDARY_VIOLATION at ({}, {})",
                maze.robot_x, maze.robot_y
            );
            break;
        }

        delay.delay_millis(100); // Control loop period
    }

    info!("ENTERING_IDLE_STATE");
    loop {
        delay.delay_millis(1000);
    }
}

