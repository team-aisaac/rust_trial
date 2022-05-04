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
    robot_command_coordinate_system_type: u8,
    target_x: i16,
    target_y: i16,
    target_angle: i16,
    vision_data_valid: bool,
    current_x: i16,
    current_y: i16,
    current_angle: i16,
    command: AisaacCommand,
    misc_byte: u8
}

impl From<AisaacFT4> for [u8; 13] {
    fn from(x: AisaacFT4) -> Self {
        let utarget_x: u16 = TryFrom::try_from(x.target_x).unwrap();
        let b0: u8 = ((x.data_type & 0x7) << 5) as u8 | ((x.robot_command_coordinate_system_type & 0b11) << 3) as u8 | if x.x_vector >= 0 {0u8} else {0b100u8} | ((utarget_x >> 11) & 0b11) as u8;
        let b1: u8 = ((utarget_x >> 3) & 0xFF) as u8;
        let utarget_y: u16 = TryFrom::try_from(x.target_y).unwrap();
        let b2: u8 = ((utarget_x & 0b111) << 5) as u8 | if x.target_y >= 0 {0u8} else {0b10000u8} | ((utarget_y >> 9) &0b1111) as u8;
        let b3: u8 = ((utarget_y >> 1) & 0xFF) as u8;
        let utarget_angle: u16 = TryFrom::try_from(x.target_angle).unwrap();
        let b4: u8 = ((utarget_y & 0b1) << 7) as u8 | if x.target_angle >= 0 {0u8} else {0b1000000u8} | ((utarget_angle >> 8) & 0b1111) as u8;
        let b5: u8 = (utarget_angle & 0xFF) as u8;
        let ucurrent_x: u16 = TryFrom::try_from(x.current_x).unwrap();
        let b6: u8 = ((x.vision_data_valid as u8) << 7) | if x.current_x >= 0 {0u8} else {0b1000000u8} | ((ucurrent_x >> 7) & 0b111111) as u8;
        let b7: u8 = ((ucurrent_x & 0b1111111) << 1) as u8 | if x.current_y >= {0u8} else {0b100u8};
        let ucurrent_y: u16 = TryFrom::try_from(x.current_y).unwrap();
        let b8: u8 = ((ucurrent_y >> 5) & 0xFF) as u8;
        let ucurrent_angle: u16 = TryFrom::try_from(x.current_angle).unwrap();
        let b9: u8 = ((ucurrent_y & 0x1F) << 3) as u8 | if x.current_angle >= 0 {0} else {0b100u8} | ((ucurrent_angle >> 9) & 0b11) as u8;
        let b10: u8 = ((ucurrent_angle >> 1) & 0xFF) as u8;
        let b11: u8 = ((ucurrent_angle & 0b1) << 7) as u8 | ((x.command.use_sensor & 0b111) << 4) as u8 | (((x.command.kick_type as u8) & 0b1) << 3) | (x.command.kick_strength & 0b111) as u8;
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
        robot_command_coordinate_system_type: 0,
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
