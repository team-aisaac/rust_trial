use protobuf::Message;

mod protos;

fn main() {
    println!("Hello, world!");

    let mut cmd = protos::aisaaccommand::AIsaacCommand::new();

    cmd.current_x = 100;
    cmd.current_y = 123456;

    let output: Vec<u8> = cmd.write_to_bytes().unwrap();

    println!("{:?}", output);

    let msg = protos::aisaaccommand::AIsaacCommand::parse_from_bytes(&output).unwrap();
    println!("cx: {}, cy: {}", msg.current_x, msg.current_y);
    println!("current angle: {}", msg.current_angle);
}
