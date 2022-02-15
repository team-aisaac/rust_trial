use std::env;
use std::path::Path;
use std::fs;

use protobuf_codegen_pure;

fn main() {
    // https://qiita.com/wada314/items/ba8a00cf87a0f349a2b9
    let root_path = env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = Path::new(&root_path).join("src");
    let proto_dir = Path::new(&root_path).join("proto");
    // https://note.katsumataryo.com/tech/2019/09/1452.html
    // https://qiita.com/kerupani129/items/37e9e04a47da195267ef
    // https://stackoverflow.com/questions/37439327/how-to-write-a-function-that-returns-vecpath
    let mut protobuf_path = vec![];
    for entry in fs::read_dir(&proto_dir).unwrap() {
        protobuf_path.push(entry.unwrap().path());
    }

    // https://docs.rs/protobuf-codegen-pure/latest/protobuf_codegen_pure/
    protobuf_codegen_pure::Codegen::new()
        .out_dir(out_dir)
        .include(proto_dir)
        .inputs(protobuf_path)
        .run()
        .expect("Failed to codegen")
}