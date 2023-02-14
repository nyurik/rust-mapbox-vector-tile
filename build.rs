use std::io::Write;

static MOD_RS: &[u8] = b"
/// Generated from src/proto/vector_tile.proto
pub mod vector_tile;
";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let proto_files = ["src/proto/vector_tile.proto"];

    for path in &proto_files {
        println!("cargo:rerun-if-changed={path}");
    }

    let out_dir = std::env::var("OUT_DIR")?;

    protobuf_codegen::Codegen::new()
        .pure()
        .out_dir(&out_dir)
        .inputs(proto_files)
        .include("src/proto")
        .run()?;

    std::fs::File::create(out_dir + "/mod.rs")?.write_all(MOD_RS)?;

    Ok(())
}
