
use protobuf_codegen::Codegen;
use std::env;
extern crate gcc;

fn main() {
    // Protobuf
    Codegen::new()
        .protoc()
        .includes(&["src/protos"])
        .input("src/protos/aisaaccommand.proto")
        .out_dir("src/protos")
        .run_from_script();
    // Build libdwa.a
    gcc::Build::new()
        .file("src/dwa2.c")
        .file("src/RaspiTrapezoidalControl.c")
        .file("src/Target_abjust.c")
        .file("src/dribble.c")
        .file("src/DWA_path_recover.c")
        .file("src/micon_wrap_kick.c")
        .file("src/tools.c")
        .file("src/TrapezoidalControl.c")
        .file("src/wrap_kick.c")
        .file("src/controller_component.c")
        .compile("dwa");
    // FFI
    let project_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={}/target/", project_dir);
    println!("cargo:rustc-link-lib=dwa");
}