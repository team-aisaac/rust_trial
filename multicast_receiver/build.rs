use std::env;
use std::path::Path;

use protobuf_codegen_pure;

fn main() {
    // https://qiita.com/wada314/items/ba8a00cf87a0f349a2b9
    let root_path = env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = Path::new(&root_path).join("src");
    // let proto_dir = Path::new(&root_path).join("proto");

    // https://docs.rs/protobuf-codegen-pure/latest/protobuf_codegen_pure/
    protobuf_codegen_pure::Codegen::new()
        .out_dir(out_dir)
        .include("proto")
        .inputs(&["proto/messages_robocup_ssl_detection.proto", "proto/messages_robocup_ssl_geometry.proto", "proto/messages_robocup_ssl_wrapper.proto"])
        .run()
        .expect("Failed to codegen")
}