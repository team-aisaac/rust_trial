use std::net::UdpSocket;
// use std::thread;
// use std::str;
use std::path::Path;
use std::env;

use rppal::uart::{Parity, Uart};

fn main() -> std::io::Result<()> {
    println!("UDP UART Relay");
    println!("Waiting at port 11312");
    // UDP
    let socket = UdpSocket::bind("127.0.0.1:11312")?;
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
                let buf = &mut buf[..buf_size];
                uart.write(&buf).expect("couldn't send uart");
            },
            Err(e) => {
                println!("couldn't receive uart message: {:?}", e);
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
