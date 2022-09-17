use protobuf_codegen::Codegen;

fn main() {
    Codegen::new()
        .protoc()
        .includes(&["src/protos"])
        .input("src/protos/aisaaccommand.proto")
        .out_dir("src/protos")
        .run_from_script();
}
