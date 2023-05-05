use std::f64::consts::PI as OtherPI;
use std::net::UdpSocket;
use std::path::Path;
use std::env;
use std::io;
use rppal::uart::{Parity, Uart};
use protobuf::Message;
use std::time::{Duration, Instant};

mod protos;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct Position {
    x: i32,
    y: i32,
    theta:i32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct F32Position {
    x: f32,
    y: f32,
    theta: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct State {
    x: f64,
    y: f64,
    theta: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct Vector2 {
    x: f32,
    y: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct Vector3 {
    x: f32,
    y: f32,
    theta: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct TrackedBall {
    pos: Vector3,
    vel: Vector3,
    visibility: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct RobotId {
    id: u32,
    team_color: u32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct TrackedRobot {
    robot_id: RobotId,
    pos: Vector2,
    orientation: f32,
    vel: Vector2,
    vel_angular: f32,
    visibility: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct MiconTrapeCon {
    jerk: f32,
    accel: f32,
    max_accel: f32,
    de_max_accel: f32,
    velocity: f32,
    max_velocity: f32,
    unit_vector_x: f32,
    unit_vector_y: f32,
    virtual_x: f32,
    virtual_y: f32,
}

#[derive(Debug, Clone, Copy)]
struct DwaRobotPath {
    x: i32, // mm
    y: i32, // mm
    theta: i32, // -180~+180 * 1000
    v_x: i32,
    v_y: i32,
    omega: i32,
    target_x: i32,
    target_y: i32,
    target_theta: i32,
    middle_target_x: i32,
    middle_target_y: i32,
}

extern "C" {
    fn execDWA(
        x: i32, y: i32, theta: i32, Vx: i32, Vy: i32, omega: i32, targetX: * mut i32, targetY: * mut i32, targetTheta: i32, middle_targetX: * mut i32, middle_targetY: * mut i32,
        numberOfObstacle: i32, ObstacleX: *const i32, ObstacleY: *const i32, ObstacleVX: *const i32, ObstacleVY: *const i32, ObstacleAX: *const i32, ObstacleAY: *const i32, prohibited_zone_ignore: bool, middle_target_flag: * mut bool,
        is_enable: * mut bool, path_enable: * mut bool, output_x: * mut i32, output_y: * mut i32, output_omega: * mut i32, output_ax: * mut i32, output_ay: * mut i32
    );

    fn DWA_path_recover(
        x: * mut f32, y: * mut f32, v_x: * mut f32, v_y: * mut f32, a_x: f32, a_y: f32, max_velocity: f32
    );

    fn micon_trapezoidal_DWA_change(
        x: i32, y: i32, v_x: i32, v_y: i32, trape_c: * mut MiconTrapeCon, target_x: i32, target_y: i32,
        trape_c_flag: bool, dwa_enable_flag: bool, dwa_path_enable_flag: bool
    ) -> bool;

    fn micon_trapezoidal_control(
        target_x: i32, target_y: i32, trape_c: * mut MiconTrapeCon
    );

    fn micon_trapezoidal_robotXY_vertualXY_distance_check(
        trape_c: * mut MiconTrapeCon, x: i32, y: i32
    );
}

fn escape_for_serial(input: u8, out: &mut Vec<u8>) {
    if input == 0x11 {
        out.push(0x7Du8);
        out.push(0x31u8);
    } else if input == 0x13 {
        out.push(0x7Du8);
        out.push(0x33u8);
    } else if input == 0x7D {
        out.push(0x7Du8);
        out.push(0x5Du8);
    } else if input == 0x7E {
        out.push(0x7Du8);
        out.push(0x5Eu8);
    } else {
        out.push(input);
    }
}

// fn serialize_spc(current_pos: protos::aisaaccommand::Position, move_vec: protos::aisaaccommand::Velocity, target_pos: protos::aisaaccommand::Position, kick: protos::aisaaccommand::Kick, robot_command_coordinate_system_type: protos::aisaaccommand::RobotCommandCoordinateSystemType, vision_data_valid: bool) -> Vec<u8> {
//     let mut cmd = protos::aisaaccommand::SpcCommand::new();
//     cmd.current_pos = protobuf::MessageField::some(current_pos);
//     cmd.move_vec = protobuf::MessageField::some(move_vec);
//     cmd.target_pos = protobuf::MessageField::some(target_pos);
//     cmd.kick = protobuf::MessageField::some(kick);
//     cmd.robot_command_coordinate_system_type = protobuf::EnumOrUnknown::new(robot_command_coordinate_system_type);
//     cmd.vision_data_valid = vision_data_valid;
//     return cmd.write_to_bytes().unwrap();
// }

// fn serialze_rpi(target_pos: protos::aisaaccommand::Position, dwa_result_valid: bool, path_enable: bool, dwa_result: protos::aisaaccommand::DwaResult) -> Vec<u8> {
//     let mut cmd = protos::aisaaccommand::RaspiCommand::new();
//     cmd.target_pos = protobuf::MessageField::some(target_pos);
//     cmd.dwa_result_valid = dwa_result_valid;
//     cmd.path_enable = path_enable;
//     cmd.dwa_result = protobuf::MessageField::some(dwa_result);
//     return cmd.write_to_bytes().unwrap();
// }


fn main() -> std::io::Result<()> {
    println!("UDP UART convert DWA version");
    println!("Waiting at port 11312");
    // UDP wait
    let socket = UdpSocket::bind("0.0.0.0:11312")?; // INADDR_ANY
    socket.set_nonblocking(true).unwrap();
    let mut buf = [0; 2048];
    // UART
    let serial_device_path = Path::new("/dev/ttyAMA1");   // Use uart2
    let mut uart = Uart::with_path(serial_device_path, 115_200, Parity::None, 8, 1).unwrap();

    // args
    let mut hex_mode = false;
    let args: Vec<String> = env::args().collect();
    for arg in args {
        if arg == "-b" {
            hex_mode = true;
            println!("Hex mode: On");
        }
    }

    // for receiving Message
    let mut received_msg_index: u8 = 0;
    let mut received_msg_length: u8 = 0;
    let mut received_msg_checksum: u8 = 0;
    let mut received_msg_escape_flag: bool = false;
    let mut received_msg: Vec<u8> = Vec::new();

    // DWA：保持する変数
    let mut current_position = Position { x: 0, y: 0, theta: 0 };
    let mut _dwa_robot: DwaRobotPath = DwaRobotPath { x: 0, y: 0, theta: 0, v_x: 0, v_y: 0, omega: 0, target_x: 0, target_y: 0, target_theta: 0, middle_target_x: 0, middle_target_y: 0 };
    let mut _trape_control_flag = false;
    let mut _trape_c: MiconTrapeCon = MiconTrapeCon { jerk: 0.0, accel: 0.0, max_accel: 0.0, de_max_accel: 0.0, velocity: 0.0, max_velocity: 0.0, unit_vector_x: 0.0, unit_vector_y: 0.0, virtual_x: 0.0, virtual_y: 0.0 };
    let base_time = Instant::now();
    let dwa_duration = Duration::new(0, 10000000);  // = 10ms = 100Hz
    let mut next_dwa_time = base_time + dwa_duration;
    let mut goal_pose = State { x: 0.0, y: 0.0, theta: 0.0 };
    let mut prohibited_zone_ignore = false;
    let mut middle_goal_pose = State { x: 0.0, y: 0.0, theta: 0.0 };
    const ROBOT_POSITION_RESET_DISTANCE: f64 = 700.0;
    const ROBOT_NUM: i32 = 16;
    const DWA_ROBOTXY_VIRTUALXY_DISTANCE_CHECK: f64 = 500.0;    // DWA中に仮の目標値とロボットの位置が離れられる最大値
    const RASPI_TIME_STEP: f64 = 0.01;  // 100Hz
    const MICON_TIME_STEP: f64 = 0.001; // 1000Hz
    const ROBOT_MAX_VEL: f32 = 2000.0;  // ロボットの最大速度(mm/s)

    // SSL Vision Data
    let mut robot_oriented_ball_position = F32Position { x: 0.0, y: 0.0, theta: 0.0 };
    let mut ball = TrackedBall { pos: Vector3 { x: 0.0, y: 0.0, theta: 0.0 }, vel: Vector3 { x:0.0, y: 0.0, theta: 0.0 }, visibility: 0.0 };
    let mut _number_of_obstacles = 0;
    let mut _obs_x = [0; 2 * ROBOT_NUM as usize];
    let mut _obs_y = [0; 2 * ROBOT_NUM as usize];
    let mut _obs_v_x = [0; 2 * ROBOT_NUM as usize];
    let mut _obs_v_y = [0; 2 * ROBOT_NUM as usize];
    let mut _obs_a_x = [0; 2 * ROBOT_NUM as usize];
    let mut _obs_a_y = [0; 2 * ROBOT_NUM as usize];

    loop {
        // UDP Rx
        // StrategyPC -> RasPi -> STM32
        match socket.recv_from(&mut buf) {
            Ok((buf_size, src_addr)) => {
                println!("src address: {:?}", src_addr);

                /*
                |IP|UDP|Version (2Byte)|RobotID (1Byte)|SequenceNo (1Byte)|DataType (1Byte)|Data|
                */
                println!("Protocon version: {}.{}", buf[0], buf[1]);
                println!("Robot ID: {}", buf[2]);
                println!("Sequence No: {}", buf[3]);
                let data_type = buf[4] >> 5;
                let sub_data_type = buf[4] & 0b11111;
                println!("Data type: {}, {}", data_type, sub_data_type);
                let serealized = &buf[5..buf_size];

                // データ型に応じた処理
                match sub_data_type {
                    0 => {  // from Strategy PC (5, 0)
                        let received_cmd = protos::aisaaccommand::SpcCommand::parse_from_bytes(&serealized).unwrap();
                        // Pass-through
                        let serialized_data = received_cmd.write_to_bytes().unwrap();
                        let mut escaped_buf: Vec<u8> = vec![0x7Eu8];        // Start delimiter
                        escape_for_serial(serialized_data.len() as u8 + 1, &mut escaped_buf);    // Length
                        let mut check_sum = 0;
                        escape_for_serial(0b10100000, &mut escaped_buf);    // Command: Data Type 5, 1
                        for x in serialized_data {
                            check_sum = (check_sum as u16 + x as u16) as u8;
                            escape_for_serial(x, &mut escaped_buf);
                        }
                        check_sum = 0xFFu8 - check_sum;
                        escape_for_serial(check_sum, &mut escaped_buf);     // Checksum
                        uart.write(&escaped_buf).expect("couldn't send uart");
                        
                        // DWA: 戦略PCから受け取る変数
                        goal_pose = State { x: received_cmd.goal_pose.x as f64, y: received_cmd.goal_pose.y as f64, theta: received_cmd.goal_pose.theta as f64 };
                        prohibited_zone_ignore = received_cmd.prohibited_zone_ignore;
                        // let mut middle_target_flag = received_cmd.middle_target_flag;
                        middle_goal_pose = State { x: received_cmd.middle_goal_pose.x as f64, y: received_cmd.middle_goal_pose.y as f64, theta: received_cmd.middle_goal_pose.theta as f64 };
                        // https://doc.rust-lang.org/nomicon/ffi.html#creating-a-safe-interface
                    },
                    2 => {  // from SSL Vision (5, 2)
                        // Send to STM32
                        let mut escaped_buf: Vec<u8> = vec![0x7Eu8];    // Start delimiter
                        escape_for_serial( serealized.len() as u8, &mut escaped_buf );    // Length
                        let mut check_sum = 0;
                        escape_for_serial( 0b10100010, &mut escaped_buf );  // Vision: Data Type: 5, 2
                        // https://zenn.dev/toga/books/rust-atcoder-old/viewer/21-slice#for-%E5%BC%8F
                        for x in &serealized[..] {
                            check_sum = ( check_sum as u16 + *x as u16 ) as u8;
                            escape_for_serial( *x, &mut escaped_buf );
                        }
                        check_sum = 0xFFu8 - check_sum;
                        escape_for_serial( check_sum, &mut escaped_buf );   // Checksum
                        uart.write(&escaped_buf).expect("Couldn't send uart.");

                        // for Compute
                        let ssl_vision_data = protos::aisaaccommand::VisionData::parse_from_bytes(&serealized).unwrap();
                        // Current position data 
                        current_position = Position { x: ssl_vision_data.own_machine_position.x, y: ssl_vision_data.own_machine_position.y, theta: ssl_vision_data.own_machine_position.theta };
                        
                        let number_of_obstacles = ssl_vision_data.obstacles.len() as i8;
                        println!("{}", number_of_obstacles);
                        let mut obstacles_position: Vec<Position> = Vec::new();
                        // obstacles_position.clear();
                        _number_of_obstacles = 0;

                        for obstacle in ssl_vision_data.obstacles {
                            obstacles_position.push(Position { x: obstacle.x, y: obstacle.y, theta: obstacle.theta });
                            _obs_x[_number_of_obstacles] = obstacle.x;   // mm
                            _obs_y[_number_of_obstacles] = obstacle.y;   // mm
                            _obs_v_x[_number_of_obstacles] = obstacle.theta; // mm/s
                            _obs_v_y[_number_of_obstacles] = 0; // mm/s
                            _obs_a_x[_number_of_obstacles] = 0; // mm/s^2
                            _obs_a_y[_number_of_obstacles] = 0; // mm/s^2
                            _number_of_obstacles = _number_of_obstacles + 1;
                        }

                        ball.pos = Vector3 { x: ssl_vision_data.ball_position.x as f32, y: ssl_vision_data.ball_position.y as f32, theta: ssl_vision_data.ball_position.theta as f32 };

                        // ボールのロボット座標の算出(ロボットはボールセンサとカメラ情報から算出する)
                        robot_oriented_ball_position = F32Position { x: (ball.pos.x - current_position.x as f32)*(-current_position.theta as f32).cos() - (ball.pos.y - current_position.y as f32)* (-current_position.theta as f32).sin() , y: (ball.pos.x as f32 - current_position.x as f32)*(-current_position.theta as f32).sin() + (ball.pos.y as f32 - current_position.y as f32)*(-current_position.theta as f32).cos(), theta: 0.0};

                    },
                    _ => {
                        // Other sub types
                    }
                }
            },  // Error handling
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {},
            Err(e) => {
                println!("couldn't receive UDP message: {:?}", e);
            }
        }

        // UART Rx
        // STM32 -> RasPi
        let mut uart_rx_buffer = [0u8, 1];
        if uart.read(&mut uart_rx_buffer).expect("couldn't receive from uart") > 0 {
            let mut rx_msg_tmp = uart_rx_buffer[0];
            
            let mut sub_data_type = 0;
            // let mut unescaped_buffer: Vec<u8> = Vec::new();
            if rx_msg_tmp == 0x7Eu8 {   // Header
                received_msg_index = 0;
                received_msg_length = 0;
                received_msg_checksum = 0;
                received_msg_escape_flag = false;
                received_msg.clear();
            } else if rx_msg_tmp == 0x7Du8 {    // Escape Flag
                received_msg_escape_flag = true;
            } else {
                if received_msg_escape_flag {
                    received_msg_escape_flag = false;
                    rx_msg_tmp = rx_msg_tmp ^ 0x20u8;   // Unescape
                }
                if received_msg_length == 0 {
                    received_msg_length = rx_msg_tmp;
                } else if received_msg_index == received_msg_length {
                    if (received_msg_checksum + rx_msg_tmp) == 0xFFu8 {
                        // Received whole message
                        sub_data_type = received_msg[0];
                    } else {
                        // Checksum error
                    }
                } else {
                    received_msg.push(rx_msg_tmp);
                    received_msg_checksum = (received_msg_checksum as u16 + rx_msg_tmp as u16) as u8;
                }
            }

            if sub_data_type == 3 {
                // Show Message
                for c in &received_msg[1..] {
                    if hex_mode {
                        print!("{:X} ", c);
                    } else {
                        print!("{}", *c as char);
                    }
                }
            }            
        }

        // Timer DWA 100Hz
        if Instant::now() > next_dwa_time {
            next_dwa_time = next_dwa_time + dwa_duration;

            let mut next_goal_pose = State { x: 0.0, y: 0.0, theta: 0.0 };  // m
            let mut _dribble_con_flag = false;

            // Receive from Strategy PC
            // - goal: https://github.com/SSL-Roots/consai_ros2/blob/b5256af08073560abad72e4371d8e2e2619b56ad/consai_msgs/action/RobotControl.action
            // - my_robot: TrackedRobot
            // - goal_pose: State
            // - kick_power: double 10.0
            // - dribble_power: double 0.0
            // Receive from SSL Vision
            // - ball info
            // - my robot info
            // - obstacles info

            let mut _middle_target_flag = false;
            {
                _dwa_robot.target_x = goal_pose.x as i32;   // mm
                _dwa_robot.target_y = goal_pose.y as i32;   // mm
                _dwa_robot.target_theta = (goal_pose.theta * 18000.0 / OtherPI) as i32;
                _dwa_robot.middle_target_x = middle_goal_pose.x as i32;
                _dwa_robot.middle_target_y = middle_goal_pose.y as i32;
                let _robot_virtual_goal_distance = ((_dwa_robot.x - current_position.x) as f64).hypot((_dwa_robot.y - current_position.y) as f64);

                if ROBOT_POSITION_RESET_DISTANCE < _robot_virtual_goal_distance {
                    _dwa_robot.x = current_position.x;
                    _dwa_robot.y = current_position.y;
                    _dwa_robot.theta = current_position.theta;
                    // _dwa_robot.v_x = current_
                    // _dwa_robot.v_y
                    // _dwa_robot.omega
                }
                
                let mut _is_dwa_enable = false;
                let mut _path_enable = false;
                let mut _output_v_x = 0;
                let mut _output_v_y = 0;
                let mut _output_omega = 0;
                let mut _output_a_x = 0;
                let mut _output_a_y = 0;
                unsafe {
                    execDWA(_dwa_robot.x, _dwa_robot.y, _dwa_robot.theta, _dwa_robot.v_x, _dwa_robot.v_y, _dwa_robot.omega,
                        &mut _dwa_robot.target_x, &mut _dwa_robot.target_y, _dwa_robot.target_theta, &mut _dwa_robot.middle_target_x, &mut _dwa_robot.middle_target_y,
                        _number_of_obstacles as i32, _obs_x.as_ptr(), _obs_y.as_ptr(), _obs_v_x.as_ptr(), _obs_v_y.as_ptr(), _obs_a_x.as_ptr(), _obs_a_y.as_ptr(), prohibited_zone_ignore,
                        &mut _middle_target_flag, &mut _is_dwa_enable, &mut _path_enable, &mut _output_v_x, &mut _output_v_x, &mut _output_omega, &mut _output_a_x, &mut _output_a_y);
                }

                let mut _next_goal_pose_x = _dwa_robot.x as f32;
                let mut _next_goal_pose_y = _dwa_robot.y as f32;
                let mut _robot_virtual_vel_x = _dwa_robot.v_x as f32;
                let mut _robot_virtual_vel_y = _dwa_robot.v_y as f32;

                if _is_dwa_enable == true && _path_enable == true {
                    // DWAを行う
                    for _i in 0..(RASPI_TIME_STEP/MICON_TIME_STEP) as i32 {
                        unsafe {
                            DWA_path_recover(&mut _next_goal_pose_x, &mut _next_goal_pose_y, &mut _robot_virtual_vel_x, &mut _robot_virtual_vel_y,
                                _output_a_x as f32, _output_a_y as f32, ROBOT_MAX_VEL);
                        }
                        let _robot_dwa_goal_distance = (next_goal_pose.x - (current_position.x as f64) * 1000.0).hypot(next_goal_pose.y - (current_position.y as f64) * 1000.0);
                        if DWA_ROBOTXY_VIRTUALXY_DISTANCE_CHECK < _robot_dwa_goal_distance {
                            break;
                        }
                    }
                    _dwa_robot.x = _next_goal_pose_x as i32;
                    _dwa_robot.y = _next_goal_pose_y as i32;
                    _dwa_robot.theta = (current_position.theta as f64 * 18000.0 / OtherPI) as i32;
                    _dwa_robot.v_x = _robot_virtual_vel_x as i32;
                    _dwa_robot.v_y = _robot_virtual_vel_y as i32;
                    // _dwa_robot.omega = current_position.vel_angular;
                    next_goal_pose.x = _next_goal_pose_x as f64;
                    next_goal_pose.y = _next_goal_pose_y as f64;
                    _trape_control_flag = false;

                } else if _is_dwa_enable == true && _path_enable == false {
                    // DWAから台形制御に移行する際に変数を設定する関数
                    unsafe {
                        _trape_control_flag = micon_trapezoidal_DWA_change(
                            _dwa_robot.x, _dwa_robot.y, _dwa_robot.v_x, _dwa_robot.v_y,
                            & mut _trape_c, _dwa_robot.target_x, _dwa_robot.target_y, _trape_control_flag,
                            _is_dwa_enable, _path_enable);
                    }
                    // 台形制御を行う
                    for _i in 0..(RASPI_TIME_STEP/MICON_TIME_STEP) as i32 {
                        unsafe {
                            micon_trapezoidal_control(_dwa_robot.target_x, _dwa_robot.target_y, &mut _trape_c);
                        }
                    }
                    // ロボットの位置と仮想目標値に大きなズレが発生した場合に補正する
                    unsafe {
                        micon_trapezoidal_robotXY_vertualXY_distance_check(&mut _trape_c, _dwa_robot.x, _dwa_robot.y);
                    }
                    _dwa_robot.x = _trape_c.virtual_x as i32;
                    _dwa_robot.y = _trape_c.virtual_y as i32;
                    _dwa_robot.theta = current_position.theta;
                    _dwa_robot.v_x = (_trape_c.unit_vector_x * _trape_c.velocity) as i32;
                    _dwa_robot.v_y = (_trape_c.unit_vector_y * _trape_c.velocity) as i32;
                    // _dwa_robot.omega = current_position.vel_angular;
                    _next_goal_pose_x = _trape_c.virtual_x;
                    _next_goal_pose_y = _trape_c.virtual_y;
                } else if _is_dwa_enable == false {
                    _trape_control_flag = false;
                    // あとで実装する (戦略PCから速度情報が与えられていないので実装できない)
                }
                next_goal_pose.x = next_goal_pose.x / 1000.0;   // mm -> m
                next_goal_pose.y = next_goal_pose.y / 1000.0;   // mm -> m
            }

            // ラズパイでの責務はここまで
            // 制御値を出力する
            let mut _command_to_stm32 = protos::aisaaccommand::DWAResult::new();
            // goal position
            let mut _dwa_goal_position = protos::aisaaccommand::Position::new();
            _dwa_goal_position.x = next_goal_pose.x * 1000.0 as i32;  // mm
            _dwa_goal_position.y = next_goal_pose.y * 1000.0 as i32;  // mm
            // _dwa_goal_position.theta = next_goal_pose.theta as i32;
            _command_to_stm32.dwa_position = protobuf::MessageField::some(_dwa_goal_position);

            // set data
            let serialized_data = _command_to_stm32.write_to_bytes().unwrap();
            let mut escaped_buf: Vec<u8> = vec![0x7Eu8];        // Start delimiter
            escape_for_serial(serialized_data.len() as u8 + 1, &mut escaped_buf);    // Length
            let mut check_sum = 0;
            escape_for_serial(0b10100001, &mut escaped_buf);    // DWA: Data Type 5, 1
            for x in serialized_data {
                check_sum = (check_sum as u16 + x as u16) as u8;
                escape_for_serial(x, &mut escaped_buf);
            }
            check_sum = 0xFFu8 - check_sum;
            escape_for_serial(check_sum, &mut escaped_buf);     // Checksum
            uart.write(&escaped_buf).expect("couldn't send uart");
        }
    }
}
// https://qiita.com/psyashes/items/8791a70ef0058c173196
// https://doc.rust-lang.org/stable/std/net/struct.UdpSocket.html#method.set_nonblocking

