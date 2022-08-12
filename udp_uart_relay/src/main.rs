use std::net::UdpSocket;
// use std::thread;
// use std::str;
use std::path::Path;
use std::env;
use std::io;

use rppal::uart::{Parity, Uart};

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

fn main() -> std::io::Result<()> {
    println!("UDP UART Relay");
    println!("Waiting at port 11312");
    // UDP
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
        match socket.recv_from(&mut buf) {
            Ok((buf_size, src_addr)) => {
                // thread::spawn(move || {
                //     let buf = &mut buf[..buf_size];
                //     let req_msg = str::from_utf8(&buf).unwrap();
                //     println!("{:}", "=".repeat(80));
                //     println!("buffer size: {:?}", buf_size);
                //     println!("src address: {:?}", src_addr);
                //     println!("request message: {:?}", req_msg);
                // });
                println!("src address: {:?}", src_addr);
                let buf2 = &mut buf[4..buf_size];

                let mut escaped_buf: Vec<u8> = vec![0x7Eu8];
                escaped_buf.push(13);
                let mut check_sum = 0;
                for x in buf2 {
                    check_sum = (check_sum as u16 + *x as u16) as u8;
                    escape_for_serial(*x, &mut escaped_buf);
                }
                check_sum = 0xFFu8 - check_sum;
                escape_for_serial(check_sum, &mut escaped_buf);

                uart.write(&escaped_buf).expect("couldn't send uart");
            },
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {},
            Err(e) => {
                println!("couldn't receive UDP message: {:?}", e);
            }
        }

        // UART Rx
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
