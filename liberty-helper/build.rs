fn main() {
    println!("cargo:rerun-if-changed=src/lib.rs");

    let mut bridge = cxx_build::bridge("src/lib.rs");

    // If you have additional C++ sources, add them here via .file()
    // bridge.file("src/native.cpp");

    bridge
        .flag_if_supported("-std=c++20")
        .compile("liberty_helper_cxx");

    // Copy generated header and .cc for consumers
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR not set");
    let out_dir = std::env::var("OUT_DIR").expect("OUT_DIR not set");

    let out_base = std::path::Path::new(&out_dir).join("cxxbridge");
    let header_src = out_base
        .join("include")
        .join("liberty-helper")
        .join("src")
        .join("lib.rs.h");
    let cc_src = out_base
        .join("sources")
        .join("liberty-helper")
        .join("src")
        .join("lib.rs.cc");

    let include_dir = std::path::Path::new(&crate_dir).join("include");
    let header_dst = include_dir.join("liberty_helper.rs.h");
    let cc_dst = include_dir.join("liberty_helper.rs.cc");
    let umbrella_header = include_dir.join("liberty_helper.hpp");

    std::fs::create_dir_all(&include_dir).expect("create include dir");

    if header_src.exists() {
        std::fs::copy(&header_src, &header_dst).expect("copy generated header");
        // Write a tiny umbrella header for nicer include name
        let content = "#pragma once\n#include \"liberty_helper.rs.h\"\n";
        std::fs::write(&umbrella_header, content).expect("write umbrella header");
        println!(
            "cargo:warning=Copied cxx header to {}",
            header_dst.display()
        );
    } else {
        println!(
            "cargo:warning=Generated cxx header not found at {}",
            header_src.display()
        );
    }

    if cc_src.exists() {
        std::fs::copy(&cc_src, &cc_dst).expect("copy generated cc");
        println!("cargo:warning=Copied cxx source to {}", cc_dst.display());
    } else {
        println!(
            "cargo:warning=Generated cxx source not found at {}",
            cc_src.display()
        );
    }
}
