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
    println!("Hello, world!");
    // https://qiita.com/psyashes/items/8791a70ef0058c173196
    
    let multicast_addr_vision_data = "224.5.23.2".to_string();
    let multicast_port_vision_data = 10006;
    let multicast_endpoint_vision_data = "0.0.0.0:".to_string() + &multicast_port_vision_data.to_string();

    println!("{}", &multicast_endpoint_vision_data);

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
