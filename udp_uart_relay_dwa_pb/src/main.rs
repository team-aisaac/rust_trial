use std::net::UdpSocket;
use std::path::Path;
use std::env;
use std::io;
use rppal::uart::{Parity, Uart};
use protobuf::Message;

mod protos;

extern "C" {
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
               is_enable: * mut bool,
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

fn serialize_to_stm32() -> Vec<u8> {
    let mut cmd = protos::aisaaccommand::RaspiCommand::new();
    let mut current_pos = protos::aisaaccommand::Position::new();
    // Set current_pos
    cmd.current_pos = protobuf::MessageField::some(current_pos);
    let mut target_pos = protos::aisaaccommand::Position::new();
    // Set target_pos
    cmd.target_pos = protobuf::MessageField::some(target_pos);
    let mut move_vec = protos::aisaaccommand::Velocity::new();
    // Set move_vec
    cmd.move_vec = protobuf::MessageField::some(move_vec);
    cmd.is_dwa_valid = true;
    let mut dwa_result = protos::aisaaccommand::DwaResult::new();
    // Set dwa
    cmd.dwa = protobuf::MessageField::some(dwa_result);
    let mut kick = protos::aisaaccommand::Kick::new();
    // Set kick
    cmd.kick = protobuf::MessageField::some(kick);
    return cmd.write_to_bytes().unwrap();
}

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
                let mut mid_tx = received_cmd.middle_target.x;
                let mut mid_ty = received_cmd.middle_target.y;
                let number_of_obstacles = received_cmd.obstacles.len() as i32;
                println!("{}", number_of_obstacles);
                let mut obstacle_x = Vec::new();
                let mut obstacle_y = Vec::new();
                let mut obstacle_vx = Vec::new();
                let mut obstacle_vy = Vec::new();
                // https://doc.rust-lang.org/nomicon/ffi.html#creating-a-safe-interface
                let prohibited_zone_ignore = received_cmd.prohibited_zone_ignore;
                let mut middle_target_flag = false;
                let mut is_enable = false;
                let mut path_enable = false;
                let mut pzs = false;
                let mut vx_out = 0;
                let mut vy_out = 0;
                let mut omega_out = 0;
                let mut ax_out = 0;
                let mut ay_out = 0;

                // DWA
                unsafe {
                    execDWA(x, y, theta, v_x, v_y, omega, &mut target_x, &mut target_y, &mut target_theta, &mut mid_tx, &mut mid_ty, number_of_obstacles, obstacle_x.as_ptr(), obstacle_y.as_ptr(), obstacle_vx.as_ptr(), obstacle_vy.as_ptr(), prohibited_zone_ignore, &mut middle_target_flag, &mut is_enable, &mut path_enable, &mut pzs, &mut vx_out, &mut vy_out, &mut omega_out, &mut ax_out, &mut ay_out);
                }

                let serialized_data = serialize_to_stm32();

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

