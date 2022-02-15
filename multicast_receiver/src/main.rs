use std::net::UdpSocket;
use std::thread;
use std::str::FromStr;
use std::net::Ipv4Addr;

use protobuf::Message;
mod messages_robocup_ssl_detection;
mod messages_robocup_ssl_geometry;
mod messages_robocup_ssl_wrapper;
use messages_robocup_ssl_wrapper::SSL_WrapperPacket;

fn main() -> std::io::Result<()> {
    println!("SSL Vision receiver");
    // https://qiita.com/psyashes/items/8791a70ef0058c173196
    
    let multicast_addr_vision_data = "224.5.23.2".to_string();
    let multicast_port_vision_data: u16 = 10006;
    let multicast_endpoint_vision_data = "0.0.0.0:".to_string() + &multicast_port_vision_data.to_string();

    println!("IPv4 socket bind with {}", &multicast_endpoint_vision_data);

    let socket = UdpSocket::bind(&multicast_endpoint_vision_data).expect("couldn't bind to address");
    let mut buf = [0; 2048];

    // Join multicast group
    socket.join_multicast_v4(
        &Ipv4Addr::from_str(&multicast_addr_vision_data).expect("couldn't generate address from str"),
        &Ipv4Addr::UNSPECIFIED
    ).expect("couldn't join multicast group");

    loop {
        match socket.recv_from(&mut buf) {
            Ok((buf_size, _src_addr)) => {
                thread::spawn(move || {
                    let _buf = &mut buf[..buf_size];
                    // https://doc.rust-lang.org/std/net/struct.UdpSocket.html
                    // https://github.com/stepancheg/rust-protobuf/blob/master/protobuf-examples/pure-vs-protoc/src/main.rs
                    // Message protobuf deserialize
                    let mut _packet = SSL_WrapperPacket::new();
                    _packet.merge_from_bytes(_buf).expect("couldn't deserialize protobuf.");
                    
                    if _packet.has_detection() {
                        println!("Detection");
                        let _detection = _packet.get_detection();
                        if _detection.has_frame_number() {
                            println!("-frame_number: {}", _detection.get_frame_number());
                        }
                        if _detection.has_t_capture() {
                            println!("-t_capture: {}", _detection.get_t_capture());
                        }
                        if _detection.has_t_sent() {
                            println!("-t_sent: {}", _detection.get_t_sent());
                        }
                        if _detection.has_camera_id() {
                            println!("-camera_id: {}", _detection.get_camera_id());
                        }
                        let _balls = _detection.get_balls();
                        for _ball in _balls.iter() {
                            println!("Ball");
                            if _ball.has_confidence() {
                                println!("-condidence: {}", _ball.get_confidence());
                            }
                            if _ball.has_area() {
                                println!("-area: {}", _ball.get_area());
                            }
                            if _ball.has_x() {
                                println!("-x: {}", _ball.get_x());
                            }
                            if _ball.has_y() {
                                println!("-y: {}", _ball.get_y());
                            }
                            if _ball.has_z() {
                                println!("-z: {}", _ball.get_z());
                            }
                            if _ball.has_pixel_x() {
                                println!("-pixel_x: {}", _ball.get_pixel_x());
                            }
                            if _ball.has_pixel_y() {
                                println!("-pixel_y: {}", _ball.get_pixel_y());
                            }
                        }
                        let _robots_yellow = _detection.get_robots_yellow();
                        for _robot in _robots_yellow.iter() {
                            println!("Robot yellow");
                            if _robot.has_confidence() {
                                println!("-confidence: {}", _robot.get_confidence());
                            }
                            if _robot.has_robot_id() {
                                println!("-robot_id: {}", _robot.get_robot_id());
                            }
                            if _robot.has_x() {
                                println!("-x: {}", _robot.get_x());
                            }
                            if _robot.has_y() {
                                println!("-y: {}", _robot.get_y());
                            }
                            if _robot.has_orientation() {
                                println!("-orientation: {}", _robot.get_orientation());
                            }
                            if _robot.has_pixel_x() {
                                println!("-pixel_x: {}", _robot.get_pixel_x());
                            }
                            if _robot.has_pixel_y() {
                                println!("-pixel_y: {}", _robot.get_pixel_y());
                            }
                            if _robot.has_height() {
                                println!("-height: {}", _robot.get_height());
                            }
                        }
                        let _robots_blue = _detection.get_robots_blue();
                        for _robot in _robots_blue.iter() {
                            println!("Robot blue");
                            if _robot.has_confidence() {
                                println!("-confidence: {}", _robot.get_confidence());
                            }
                            if _robot.has_robot_id() {
                                println!("-robot_id: {}", _robot.get_robot_id());
                            }
                            if _robot.has_x() {
                                println!("-x: {}", _robot.get_x());
                            }
                            if _robot.has_y() {
                                println!("-y: {}", _robot.get_y());
                            }
                            if _robot.has_orientation() {
                                println!("-orientation: {}", _robot.get_orientation());
                            }
                            if _robot.has_pixel_x() {
                                println!("-pixel_x: {}", _robot.get_pixel_x());
                            }
                            if _robot.has_pixel_y() {
                                println!("-pixel_y: {}", _robot.get_pixel_y());
                            }
                            if _robot.has_height() {
                                println!("-height: {}", _robot.get_height());
                            }
                        }
                    }

                    if _packet.has_geometry() {
                        println!("Geometry");
                        let _geometry = _packet.get_geometry();
                        if _geometry.has_field() {
                            let _field = _geometry.get_field();
                            if _field.has_field_length() {
                                println!("-field_length: {}", _field.get_field_length());
                            }
                            if _field.has_field_width() {
                                println!("-field_width: {}", _field.get_field_width());
                            }
                            // other fields ...
                        }
                        // if _geometry.has_calib
                        let _calibs = _geometry.get_calib();
                        for _calib in _calibs {
                            if _calib.has_camera_id() {
                                println!("-camera_id: {}", _calib.get_camera_id());
                            }
                            if _calib.has_focal_length() {
                                println!("-focal_length: {}", _calib.get_focal_length());
                            }
                            if _calib.has_principal_point_x() {
                                println!("-principal_point_x: {}", _calib.get_principal_point_x());
                            }
                            if _calib.has_principal_point_y() {
                                println!("-principal_point_y: {}", _calib.get_principal_point_y());
                            }
                            if _calib.has_distortion() {
                                println!("-distortion: {}", _calib.get_distortion());
                            }
                            // other fields ...
                        }
                        if _geometry.has_models() {
                            let _models = _geometry.get_models();
                            if _models.has_straight_two_phase() {
                                let _straight_two_pahse = _models.get_straight_two_phase(); 
                            }
                            if _models.has_chip_fixed_loss() {
                                let _chip_fixed_loss = _models.get_chip_fixed_loss();
                                if _chip_fixed_loss.has_damping_xy_first_hop() {
                                    println!("-damping_xy_first_hop: {}", _chip_fixed_loss.get_damping_xy_first_hop());
                                }
                                if _chip_fixed_loss.has_damping_xy_other_hops() {
                                    println!("-damping_xy_other_hops: {}", _chip_fixed_loss.get_damping_xy_other_hops());
                                    // multiple?
                                }
                                if _chip_fixed_loss.has_damping_z() {
                                    println!("-damping_z: {}", _chip_fixed_loss.get_damping_z());
                                }
                            }
                        }
                    }
                });
            },
            Err(e) => {
                println!("couldn't receive request: {:?}", e);
            }
        }
    }
    
    // Leave multicast group
    // socket.leave_multicast_v4(
    //     &Ipv4Addr::from_str(&multicast_addr_vision_data).expect("couldn't generate address from str"),
    //     &Ipv4Addr::UNSPECIFIED
    // ).expect("couldn't leave from multicast group");

    // Ok(())
}
