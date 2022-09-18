use std::net::UdpSocket;
use std::path::Path;
use std::env;
use std::io;
use rppal::uart::{Parity, Uart};
use protobuf::Message;

mod protos;

extern "C" {
    // fn hello_world();
    fn execDWA(x: i32,
               y: i32,
               theta: i32,
               Vx: i32,
               Vy: i32,
               omega: i32,
               targetX: * mut i32,
               targetY: * mut i32,
               targetTheta: * mut i32,
               middle_targetX: * mut i32,
               middle_targetY: * mut i32,
               numberOfObstacle: i32,
               ObstacleX: *const i32,
               ObstacleY: *const i32,
               ObstacleVX: *const i32,
               ObstacleVY: *const i32,
               prohibited_zone_ignore: bool,
               middle_target_flag: * mut bool,
               dwa_result_valid: * mut bool,
               path_enable: * mut bool,
               prohibited_zone_start: * mut bool,
               vx_out: * mut i32,
               vy_out: * mut i32,
               omega_out: * mut i32,
               ax_out: * mut i32,
               ay_out: * mut i32);
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

fn parse_protobuf_from_strategy_pc(input: [u8; 2048], buf_size: usize) -> protos::aisaaccommand::SpcCommand {
    println!("Protocon version: {}.{}", input[0], input[1]);
    println!("Robot ID: {}", input[2]);
    println!("Sequence No: {}", input[3]);
    println!("Sequence No: {}", input[4]);
    let serealized = &input[5..buf_size];

    let cmd = protos::aisaaccommand::SpcCommand::parse_from_bytes(&serealized).unwrap();
    return cmd
}

fn serialize_to_stm32(current_pos: protos::aisaaccommand::Position, move_vec: protos::aisaaccommand::Velocity, target_pos: protos::aisaaccommand::Position, dwa_result_valid: bool, path_enable: bool, dwa_result: protos::aisaaccommand::DwaResult, kick: protos::aisaaccommand::Kick) -> Vec<u8> {
    let mut cmd = protos::aisaaccommand::RaspiCommand::new();
    cmd.current_pos = protobuf::MessageField::some(current_pos);
    cmd.move_vec = protobuf::MessageField::some(move_vec);
    cmd.target_pos = protobuf::MessageField::some(target_pos);
    cmd.dwa_result_valid = dwa_result_valid;
    cmd.dwa_result = protobuf::MessageField::some(dwa_result);
    cmd.kick = protobuf::MessageField::some(kick);
    return cmd.write_to_bytes().unwrap();
}

fn main() -> std::io::Result<()> {
    println!("UDP UART convert DWA version");
    println!("Waiting at port 11312");
    // unsafe {
    //     hello_world();
    // }
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

    loop {
        // UDP Rx
        // StrategyPC -> RasPi -> STM32
        match socket.recv_from(&mut buf) {
            Ok((buf_size, src_addr)) => {
                println!("src address: {:?}", src_addr);
                let received_cmd = parse_protobuf_from_strategy_pc(buf, buf_size);

                let x: i32 = received_cmd.current_pos.x;
                let y: i32 = received_cmd.current_pos.y;
                let theta: i32 = received_cmd.current_pos.theta;
                let v_x = received_cmd.move_vec.vx;
                let v_y = received_cmd.move_vec.vy;
                let omega = received_cmd.move_vec.omega;
                let mut target_x = received_cmd.target_pos.x;
                let mut target_y = received_cmd.target_pos.y;
                let mut target_theta = received_cmd.target_pos.theta;
                let mut mid_tx = 0;
                let mut mid_ty = 0;
                let number_of_obstacles = received_cmd.obstacles.len() as i32;
                println!("{}", number_of_obstacles);
                let mut obstacle_x = Vec::new();
                let mut obstacle_y = Vec::new();
                let mut obstacle_vx = Vec::new();
                let mut obstacle_vy = Vec::new();
                for obstacle in received_cmd.obstacles {
                    obstacle_x.push(obstacle.x);
                    obstacle_y.push(obstacle.y);
                    obstacle_vx.push(obstacle.vx);
                    obstacle_vy.push(obstacle.vy);
                }
                // https://doc.rust-lang.org/nomicon/ffi.html#creating-a-safe-interface
                let prohibited_zone_ignore = received_cmd.prohibited_zone_ignore;
                let mut middle_target_flag = false;
                let mut dwa_result_valid = false;
                let mut path_enable = false;
                let mut pzs = false;
                let mut vx_out = 0;
                let mut vy_out = 0;
                let mut omega_out = 0;
                let mut ax_out = 0;
                let mut ay_out = 0;

                // DWA
                unsafe {
                    execDWA(x, y, theta, v_x, v_y, omega, &mut target_x, &mut target_y, &mut target_theta, &mut mid_tx, &mut mid_ty, number_of_obstacles, obstacle_x.as_ptr(), obstacle_y.as_ptr(), obstacle_vx.as_ptr(), obstacle_vy.as_ptr(), prohibited_zone_ignore, &mut middle_target_flag, &mut dwa_result_valid, &mut path_enable, &mut pzs, &mut vx_out, &mut vy_out, &mut omega_out, &mut ax_out, &mut ay_out);
                }

                // Send to STM32
                let mut target_pos = protos::aisaaccommand::Position::new();
                target_pos.x = target_x;
                target_pos.y = target_y;
                target_pos.theta = target_theta;
                let mut dwa_result = protos::aisaaccommand::DwaResult::new();
                dwa_result.vx = vx_out;
                dwa_result.vy = vy_out;
                dwa_result.omega = omega_out;
                dwa_result.ax = ax_out;
                dwa_result.ay = ay_out;
                
                let serialized_data = serialize_to_stm32(received_cmd.current_pos.unwrap(), received_cmd.move_vec.unwrap(), target_pos, dwa_result_valid, path_enable, dwa_result, received_cmd.kick.unwrap());

                let mut escaped_buf: Vec<u8> = vec![0x7Eu8];        // Start delimiter
                escape_for_serial(serialized_data.len() as u8, &mut escaped_buf);    // Length
                let mut check_sum = 0;
                for x in serialized_data {
                    check_sum = (check_sum as u16 + x as u16) as u8;
                    escape_for_serial(x, &mut escaped_buf);
                }
                check_sum = 0xFFu8 - check_sum;
                escape_for_serial(check_sum, &mut escaped_buf);     // Checksum

                uart.write(&escaped_buf).expect("couldn't send uart");
            },
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {},
            Err(e) => {
                println!("couldn't receive UDP message: {:?}", e);
            }
        }

        // UART Rx
        // STM32 -> RasPi
        let mut uart_rx_buffer = [0u8, 1];
        if uart.read(&mut uart_rx_buffer).expect("couldn't receive from uart") > 0 {
            if hex_mode {
                print!("{:X} ", uart_rx_buffer[0]);
            } else {
                print!("{}", uart_rx_buffer[0] as char);
            }
        }
    }
}
// https://qiita.com/psyashes/items/8791a70ef0058c173196
// https://doc.rust-lang.org/stable/std/net/struct.UdpSocket.html#method.set_nonblocking

