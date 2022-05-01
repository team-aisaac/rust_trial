use std::error::Error;
use std::time::{Duration, Instant};
use std::path::Path;
// use std::thread;
use std::env;

use rppal::uart::{Parity, Uart};
use std::convert::{From, TryFrom};

struct AisaacCommand {
    use_sensor: u8,
    kick_type: bool,
    kick_strength: u8
}

// #[derive(PartialEq, Eq, Debug)]
struct AisaacFT4 {
    data_type: u8,
    x_vector: i16,
    y_vector: i16,
    angle_type_select: bool,
    angle: u16,
    calibration_valid_flag: bool,
    calibratiob_x_pos: u16,
    calibration_y_pos: u16,
    calibration_angle: u16,
    command: AisaacCommand,
    misc_byte: u8
}

impl From<AisaacFT4> for [u8; 13] {
    fn from(x: AisaacFT4) -> Self {
        let ux_vec: u16 = TryFrom::try_from(x.x_vector).unwrap();
        let b0: u8 = ((x.data_type & 0x7) << 5) as u8 | if x.x_vector >= 0 {0u8} else {0b10000u8} | ((ux_vec >> 11) & 0b1111) as u8;
        let b1: u8 = ((ux_vec >> 3) & 0xFF) as u8;
        let uy_vec: u16 = TryFrom::try_from(x.y_vector).unwrap();
        let b2: u8 = ((ux_vec & 0b111) << 5) as u8 | if x.y_vector >= 0 {0u8} else {0b10000u8} | ((uy_vec >> 11) &0b1111) as u8;
        let b3: u8 = ((uy_vec >> 3) & 0xFF) as u8;
        let b4: u8 = ((uy_vec & 0b111) << 5) as u8 | (((x.angle_type_select as u8) << 4) & 0b10000u8) | ((x.angle >> 8) & 0b1111) as u8;
        let b5: u8 = (x.angle & 0xFF) as u8;
        let b6: u8 = ((x.calibration_valid_flag as u8) << 7) | ((x.calibratiob_x_pos >> 7) & 0x7F) as u8;
        let b7: u8 = ((x.calibratiob_x_pos & 0xFE) << 1) as u8 | ((x.calibration_y_pos >> 13) & 0b1) as u8;
        let b8: u8 = ((x.calibration_y_pos >> 5) & 0xFF) as u8;
        let b9: u8 = ((x.calibration_y_pos & 0x1F) << 3) as u8 | ((x.calibration_angle >> 9) & 0b111) as u8;
        let b10: u8 = ((x.calibration_angle >> 1) & 0xFF) as u8;
        let b11: u8 = ((x.calibration_angle & 0b1) << 7) as u8 | ((x.command.use_sensor & 0b111) << 4) as u8 | (((x.command.kick_type as u8) & 0b1) << 3) | (x.command.kick_strength & 0b111) as u8;
        let b12: u8 = x.misc_byte;
        {
            [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12]
        }
    }
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

    let acm = AisaacCommand { use_sensor: 2, kick_type: true, kick_strength: 1};    
    let aft4 = AisaacFT4 {
        data_type: 4,
        x_vector: 1700,
        y_vector: 0,
        angle_type_select: true,
        angle: 0,
        calibration_valid_flag: true,
        calibratiob_x_pos: 10,
        calibration_y_pos: 10,
        calibration_angle: 200,
        command: acm,
        misc_byte: 0};
    // let test_buf = <[u8; 13]>::From(aft4);
    let test_buf: [u8; 13] = aft4.into();
    for x in test_buf {
        print!("{:x}", x);
    }
    println!("");

    let mut escaped_buf: Vec<u8> = vec![0x7Eu8];
    escaped_buf.push(13);
    let mut check_sum = 0;
    for x in test_buf {
        check_sum = (check_sum as u16 + x as u16) as u8;
        escape_for_serial(x, &mut escaped_buf);
    }
    check_sum = 0xFFu8 - check_sum;
    escape_for_serial(check_sum, &mut escaped_buf);
    // for x in escaped_buf {
    //     print!("{:x}", x);
    // }
    // println!("");


    let mut reference_time = Instant::now();
    // let buffer: [u8; 4] = ['t' as u8, 'e' as u8, 's' as u8, 't' as u8];
    loop {
        // Rx
        let mut rx_buffer = [0u8; 1];
        if uart.read(&mut rx_buffer).expect("couldn't receive from uart") > 0 {
            if hex_mode {
                print!("{:X} ", rx_buffer[0]);
            } else {
                print!("{}", rx_buffer[0] as char);
            }
        }

        if reference_time.elapsed() > Duration::from_millis(1000) {
            reference_time = Instant::now();
            uart.write(&escaped_buf).expect("couldn't send uart");
        }
        // thread::sleep(Duration::from_millis(1000)); // wait 1s
    }
}
