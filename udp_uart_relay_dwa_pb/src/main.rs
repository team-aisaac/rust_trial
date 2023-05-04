use std::net::UdpSocket;
use std::path::Path;
use std::env;
use std::io;
use std::time::Instant;
use rppal::uart::{Parity, Uart};
use protobuf::Message;

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

extern "C" {
    fn robot_wrap_kick(
        next_goal_pose: * mut State,
        ball: TrackedBall,
        r_ball: State,
        ball_goal: State,
        my_robot: TrackedRobot,
        circumferential_error: * mut f64,
        radius_error: * mut f64,
        goal_theta: * mut f64,
        ball_kick_con_flag: * mut bool,
        robot_id: u32,
        kick_con_max_velocity_theta: * mut f32,
        free_kick_flag: bool,
        ball_target_allowable_error: i32,
        ball_kick: bool,
        ball_kick_con: * mut bool,
        ob_unit_vec_circumferential_x: * mut f32,
        ob_unit_vec_circumferential_y: * mut f32,
        ob_unit_vec_radius_x: * mut f32,
        ob_unit_vec_radius_y: * mut f32,
        wrap_kick_xy_flag: * mut bool,
        dribble_active: * mut bool);

    fn dribble(
        dribble_goal: State,
        ball: TrackedBall,
        r_ball: State,
        my_robot: TrackedRobot,
        next_goal_pose: * mut State,
        dribble_con_flag: * mut bool,
        robot_id: u32,
        dribble_complete_distance: i32,
        dribble_trape_c: * mut MiconTrapeCon,
        dribble_ball_move_flag: * const bool,
        circumferential_error: * mut f64,
        radius_error: * mut f64,
        ob_unit_vec_circumferential_x: * mut f32,
        ob_unit_vec_circumferential_y: * mut f32,
        ob_unit_vec_radius_x: * mut f32,
        ob_unit_vec_radius_y: * mut f32,
        dribble_active: * mut bool);

    fn decide_next_goal_xy(
        goal_pose: State,
        middle_goal_pose: * mut State,
        next_goal_pose: * mut State,
        prohibited_zone_ignore: bool,
        middle_target_flag: * mut bool,
        robot_id: u32,
        my_robot: TrackedRobot,
        team_is_yellow: bool,
        trape_control_flag: * mut bool,
        trape_c: * mut MiconTrapeCon);
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

    // SSL Vision Data
    let mut robot_oriented_ball_position = F32Position { x: 0.0, y: 0.0, theta: 0.0 };
    let mut ball = TrackedBall { pos: Vector3 { x: 0.0, y: 0.0, theta: 0.0 }, vel: Vector3 { x:0.0, y: 0.0, theta: 0.0 }, visibility: 0.0 };
    // Variables for DWA
    let mut current_position = Position { x: 0, y: 0, theta: 0 };

    let mut r_ball = State { x: 0.0, y: 0.0, theta: 0.0 };
    let mut ball_goal = State { x: 0.0, y: 0.0, theta: 0.0 };
    let robot_id:u32 = 0;
    let my_robot_team_color = 1;    // Yellow
    let mut my_robot = TrackedRobot { robot_id: RobotId { id: robot_id, team_color: my_robot_team_color }, pos: Vector2 { x: 0.0, y: 0.0 }, orientation: 0.0, vel: Vector2 { x: 0.0, y: 0.0 }, vel_angular: 0.0, visibility: 0.0 };

    // for fn robot_wrap_kick
    let mut circumferential_error: f64 = 0.0;
    let mut radius_error:f64 = 0.0;
    let mut goal_theta:f64 = 0.0;
    let mut kick_con_max_velocity_theta:f32 = 0.0;
    let mut free_kick_flag = false;
    let mut ball_target_allowable_error: i32 = 0;
    let mut ob_unit_vec_circumferential_x: f32 = 0.0;
    let mut ob_unit_vec_circumferential_y: f32 = 0.0;
    let mut ob_unit_vec_radius_x: f32 = 0.0;
    let mut ob_unit_vec_radius_y: f32 = 0.0;

    let dribble_complete_distance = 0;
    let team_is_yellow = true;

    let mut trape_c:Vec<MiconTrapeCon> = Vec::new();

    // a
    let mut trape_control_flag:Vec<bool> = Vec::new();

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
                    0 => {  // from Strategy PC
                        let received_cmd = protos::aisaaccommand::SpcCommand::parse_from_bytes(&serealized).unwrap();
                        // Pass-through
                        // set data
                        {
                            let serialized_data = received_cmd.write_to_bytes().unwrap();
                            let mut escaped_buf: Vec<u8> = vec![0x7Eu8];        // Start delimiter
                            escape_for_serial(serialized_data.len() as u8 + 1, &mut escaped_buf);    // Length
                            let mut check_sum = 0;
                            escape_for_serial(0b10100000, &mut escaped_buf);    // Data Type 5, 0
                            for x in serialized_data {
                                check_sum = (check_sum as u16 + x as u16) as u8;
                                escape_for_serial(x, &mut escaped_buf);
                            }
                            check_sum = 0xFFu8 - check_sum;
                            escape_for_serial(check_sum, &mut escaped_buf);     // Checksum
                            uart.write(&escaped_buf).expect("couldn't send uart");
                        }
                        
                        // Data
                        let coordinate_type = received_cmd.robot_command_type;
                        let goal_pose = State { x: received_cmd.goal_pose.x as f64, y: received_cmd.goal_pose.y as f64, theta: received_cmd.goal_pose.theta as f64 };
                        let prohibited_zone_ignore = received_cmd.prohibited_zone_ignore;
                        let mut middle_target_flag = received_cmd.middle_target_flag;
                        let mut middle_goal_pose = State { x: received_cmd.middle_goal_pose.x as f64, y: received_cmd.middle_goal_pose.y as f64, theta: received_cmd.middle_goal_pose.theta as f64 };
                        let dribble_msg = received_cmd.dribble;
                        let kick = received_cmd.kick;
                        // https://doc.rust-lang.org/nomicon/ffi.html#creating-a-safe-interface
                        
                        let mut next_goal_pose = State { x: 0.0, y: 0.0, theta: 0.0 };
                        let mut ball_wrap_pid = false;
                        let mut position_pid = false;
                        let mut dribble_con_flag:Vec<bool> = Vec::new();
                        let mut dribble_ball_move_flag:Vec<bool> = Vec::new();
                        let mut dribble_power = 0.0;
                        let mut kick_power = 0;

                        // Pass-through
                        let mut ball_kick_con_flag: Vec<bool> = Vec::new();

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

                        if kick.ball_kick_active {  // ボールを蹴るための位置移動を行うかを判定(戦略PCから送信)
                            // ボールセンサに反応するかを判定(ロボットに実装する場合はボールセンサでボールの位置が取れるかで判定する)
                            if ((-0.06 <= robot_oriented_ball_position.y && robot_oriented_ball_position.y <= 0.06) && (0.08 <= robot_oriented_ball_position.x && robot_oriented_ball_position.x <= 0.13)) || ball_kick_con_flag[0] {
                                trape_control_flag[0] = false; // DWAの台形制御のフラグ
                                // ボールセンサに反応だないときはカメラの情報、ボールセンサが反応しているときはその値を使用する
                                let mut ball_kick_con = false;  // ボールを蹴る動作を実行するかを判定するフラグ
                                let mut _wrap_kick_con = false;
                                let mut wrap_kick_xy_flag = false;
                                let mut dribble_active = false;
                                let ball_kick = false;

                                // fn robot_wrap_kick
                                unsafe {
                                    robot_wrap_kick(&mut next_goal_pose, ball, r_ball, ball_goal, my_robot, &mut circumferential_error, &mut radius_error, &mut goal_theta, ball_kick_con_flag.as_mut_ptr(), robot_id, &mut kick_con_max_velocity_theta, free_kick_flag, ball_target_allowable_error, ball_kick, &mut ball_kick_con, &mut ob_unit_vec_circumferential_x, &mut ob_unit_vec_circumferential_y, &mut ob_unit_vec_radius_x, &mut ob_unit_vec_radius_y, &mut wrap_kick_xy_flag, &mut dribble_active);
                                }

                                if dribble_active {
                                    dribble_power = 0.1;
                                } else {
                                    dribble_power = 0.0;
                                }

                                // Pack in cmd
                                if wrap_kick_xy_flag {
                                    ball_wrap_pid = false;
                                    position_pid = true;
                                } else {
                                    ball_wrap_pid = true;
                                    position_pid = false;
                                }

                                if ball_kick_con {   // ボールを蹴る処理を実行
                                    kick_power = kick.kick_power;
                                }

                                next_goal_pose.theta = goal_pose.theta as f64;
                            }
                        } else {
                            ball_kick_con_flag[0] = false;
                        }

                        if dribble_msg.dribble_state {
                            // ボールセンサに反応するかを判定(ロボットに実装する場合はボールセンサでボールの位置が取れるかで判定する)
                            if ((-0.06 <= robot_oriented_ball_position.y && robot_oriented_ball_position.y <= 0.06) && (0.08 <= robot_oriented_ball_position.x && robot_oriented_ball_position.x <= 0.13)) || ball_kick_con_flag[0] {
                                trape_control_flag[0] = true;
                                let mut dribble_active = true;

                                let mut dribble_goal = State { x: 0.0, y: 0.0, theta: 0.0 };
                                let dribble_complete_distance:i32 = 0;
                                let mut dribble_trape_c:Vec<MiconTrapeCon> = Vec::new();

                                // fn dribble
                                unsafe {
                                    dribble(dribble_goal, ball, r_ball, my_robot, &mut next_goal_pose, dribble_con_flag.as_mut_ptr(), robot_id, dribble_complete_distance, dribble_trape_c.as_mut_ptr(), dribble_ball_move_flag.as_mut_ptr(), &mut circumferential_error, &mut radius_error, &mut ob_unit_vec_circumferential_x, &mut ob_unit_vec_circumferential_y, &mut ob_unit_vec_radius_x, &mut ob_unit_vec_radius_y, &mut dribble_active);
                                }

                                if dribble_active {
                                    dribble_power = 0.1;
                                } else {
                                    dribble_power = 0.0;
                                }
                                // command_to_stm32.dribble.dribble_power = dribble_power;

                                if dribble_ball_move_flag[0] {
                                    ball_wrap_pid = false;
                                    position_pid = true;
                                } else {
                                    ball_wrap_pid = true;
                                    position_pid = false;
                                }
                            } else {
                                dribble_con_flag[0] = false;
                                dribble_ball_move_flag[0] = false;
                            }
                        } else {
                            dribble_con_flag[0] = false;
                            dribble_ball_move_flag[0] = false;
                        }

                        if ball_kick_con_flag[0] == false || dribble_con_flag[0] == false {
                            dribble_ball_move_flag[0] = false;
                            ball_wrap_pid = false;
                            position_pid = true;
                            dribble_power = 0.0;
                            // 算出された目標点まで移動するために、次のループまでの目標点を決定する
                            // fn decide_next_goal_xy
                            unsafe {
                                decide_next_goal_xy(goal_pose, &mut middle_goal_pose, &mut next_goal_pose, prohibited_zone_ignore, &mut  middle_target_flag, robot_id, my_robot, team_is_yellow, trape_control_flag.as_mut_ptr(), trape_c.as_mut_ptr());
                            }
                        }

                        // ラズパイでの責務はここまで
                        // 制御値を出力する
                        let mut _command_to_stm32 = protos::aisaaccommand::RaspiCommand::new();
                        // goal pose velocity;
                        let mut _dwa_velocity = protos::aisaaccommand::Velocity::new();
                        _dwa_velocity.vx = middle_goal_pose.x as i32;
                        _dwa_velocity.vy = middle_goal_pose.y as i32;
                        _command_to_stm32.goal_pose_velocity = protobuf::MessageField::some(_dwa_velocity);
                        // dribble
                        let mut _dwa_dribble = protos::aisaaccommand::Dribble::new();
                        _command_to_stm32.dribble = protobuf::MessageField::some(_dwa_dribble);
                        // kick
                        let mut _dwa_kick = protos::aisaaccommand::Kick::new();
                        _command_to_stm32.kick = protobuf::MessageField::some(_dwa_kick);

                        // set data
                        let serialized_data = _command_to_stm32.write_to_bytes().unwrap();
                        let mut escaped_buf: Vec<u8> = vec![0x7Eu8];        // Start delimiter
                        escape_for_serial(serialized_data.len() as u8 + 1, &mut escaped_buf);    // Length
                        let mut check_sum = 0;
                        escape_for_serial(0b10100000, &mut escaped_buf);    // Data Type 5, 0
                        for x in serialized_data {
                            check_sum = (check_sum as u16 + x as u16) as u8;
                            escape_for_serial(x, &mut escaped_buf);
                        }
                        check_sum = 0xFFu8 - check_sum;
                        escape_for_serial(check_sum, &mut escaped_buf);     // Checksum
                        uart.write(&escaped_buf).expect("couldn't send uart");

                    },
                    1 => {  // from SSL Vision
                        // Send to STM32
                        let mut escaped_buf: Vec<u8> = vec![0x7Eu8];    // Start delimiter
                        escape_for_serial( serealized.len() as u8, &mut escaped_buf );    // Length
                        let mut check_sum = 0;
                        escape_for_serial( 0b10100001, &mut escaped_buf );  // Data Type: 5, 1
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
                        // let ball_position = Position { x: ssl_vision_data.ball_position.x, y: ssl_vision_data.ball_position.y, theta: ssl_vision_data.ball_position.theta };
                        let number_of_obstacles = ssl_vision_data.obstacles.len() as i8;
                        println!("{}", number_of_obstacles);
                        let mut obstacles_position: Vec<Position> = Vec::new();
                        obstacles_position.clear();
                        for obstacle in ssl_vision_data.obstacles {
                            obstacles_position.push(Position { x: obstacle.x, y: obstacle.y, theta: obstacle.theta })
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
    }
}
// https://qiita.com/psyashes/items/8791a70ef0058c173196
// https://doc.rust-lang.org/stable/std/net/struct.UdpSocket.html#method.set_nonblocking

