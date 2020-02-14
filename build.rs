

//extern crate gcc;
use std::env;

fn main() {
//    let out_dir = env::var("OUT_DIR").unwrap();
    let work_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={}/src/lib/",  work_dir);
    #[cfg(feature = "stm32f4x")]
    println!("cargo:rustc-link-lib=cmsis_rtos2_stm32f4");
    #[cfg(feature = "stm32h7x")]
    println!("cargo:rustc-link-lib=cmsis_rtos2_stm32h7");
}

