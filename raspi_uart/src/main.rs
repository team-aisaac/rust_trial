use std::error::Error;
use std::time::Duration;
use std::path::Path;
use std::thread::sleep;

use rppal::uart::{Parity, Uart};

fn main() -> Result<(), Box<dyn Error>> {
    println!("Hello, world!");
    // https://github.com/golemparts/rppal/blob/master/examples/uart_blocking_read.rs
    // https://doc.rust-jp.rs/rust-by-example-ja/std_misc/path.html
    // https://stackoverflow.com/questions/70084581/why-does-using-rppals-uart-to-read-a-software-serial-port-on-the-raspberry-pi-n
    let path = Path::new("/dev/ttyAMA1");   // Use uart2
    let mut uart = Uart::with_path(path, 115_200, Parity::None, 8, 1).unwrap();

    // uart.set_read_mode(1, Duration::default())?;
    // let mut buffer = [0u8; 1];
    let buffer: [u8; 4] = ['t' as u8, 'e' as u8, 's' as u8, 't' as u8];
    loop {
        uart.write(&buffer).expect("couldn't send uart");
        sleep(Duration::from_millis(1000)); // wait 1s
    }
}
