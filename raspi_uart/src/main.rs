use std::error::Error;
use std::time::{Duration, Instant};
use std::path::Path;
// use std::thread;
use std::env;

use rppal::uart::{Parity, Uart};
// use std::convert::{From, Into};

fn main() -> Result<(), Box<dyn Error>> {
    println!("Hello, world!");
    // https://github.com/golemparts/rppal/blob/master/examples/uart_blocking_read.rs
    // https://doc.rust-jp.rs/rust-by-example-ja/std_misc/path.html
    // https://stackoverflow.com/questions/70084581/why-does-using-rppals-uart-to-read-a-software-serial-port-on-the-raspberry-pi-n
    let path = Path::new("/dev/ttyAMA1");   // Use uart2
    let mut uart = Uart::with_path(path, 115_200, Parity::None, 8, 1).unwrap();

    let mut hex_mode = false;
    let args: Vec<String> = env::args().collect();
    for arg in args {
        if arg == "-b" {
            hex_mode = true;
            println!("Hex mode: On");
        }
    }

    // uart.set_read_mode(1, Duration::default())?;
    // let mut buffer = [0u8; 1];
    let mut reference_time = Instant::now();
    let buffer: [u8; 4] = ['t' as u8, 'e' as u8, 's' as u8, 't' as u8];
    loop {
        // Rx
        let mut rx_buffer = [0u8; 1];
        if uart.read(&mut rx_buffer).expect("couldn't receive from uart") > 0 {
            if hex_mode {
                println!("{:X} ", rx_buffer[0]);
            } else {
                println!("{}", rx_buffer[0] as char);
            }
        }

        if reference_time.elapsed() > Duration::from_millis(1000) {
            reference_time = Instant::now();
            uart.write(&buffer).expect("couldn't send uart");
        }
        // thread::sleep(Duration::from_millis(1000)); // wait 1s
    }
}

// struct AisaacCommand {
//     UseSensor: u8,
//     KickType: bool,
//     KickStrength: u8
// }

// #[derive(PartialEq, Eq, Debug)]
// struct AisaacFT4 {
//     DataType: u8,
//     XVector: i16,
//     YVector: i16,
//     TOFlag: bool,
//     Angle: u16,
//     CalibValidFlag: bool,
//     CalibXPos: u16,
//     CalibYPos: u16,
//     Command: AisaacCommand,
//     MiscByte: u8
// }

// impl From<Color> for [8u; 13] {
//     fn from(x: AisaacFT4) -> Self {
//         let b0: u8 = ((x.DataType & 0x7) << 5) as u8 | ((x.XVector >> ))

//         [8u; 13] {

//         }
//     }
// }
