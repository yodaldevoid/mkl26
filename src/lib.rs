#![allow(private_no_mangle_statics)]
#![feature(asm)]
#![feature(compiler_builtins_lib)]
#![feature(core_intrinsics)]
#![feature(lang_items)]
#![feature(linkage)]
#![feature(used)]
#![no_std]

extern crate bit_field;
extern crate compiler_builtins;
extern crate volatile_register;

use core::slice;

pub mod adc;
pub mod mcg;
pub mod osc;
pub mod port;
pub mod read;
pub mod sim;
pub mod uart;
pub mod watchdog;
pub mod interrupts;
pub mod exceptions;

extern {
    static mut _bss_start: u8;
    static mut _bss_end: u8;

    pub fn _stack_top();

    fn main(argc: isize, argv: *const *const u8) -> isize;
}

#[link_section = ".vector_table.reset_vector"]
#[no_mangle]
#[used]
static RESET_VECTOR: unsafe extern fn() -> ! = reset_handler;

/// The reset handler
///
/// This is the entry point of all programs
#[link_section = ".reset_handler"]
unsafe extern "C" fn reset_handler() -> ! {
    setup_bss();

    //TODO: FPU support
    // Neither `argc` or `argv` make sense in bare metal context so we
    // just stub them
    main(0, ::core::ptr::null());

    // If `main` returns, then we go into "reactive" mode and simply attend
    // interrupts as they occur.
    loop {
        asm!("wfi" :::: "volatile");
    }
}

#[lang = "start"]
extern "C" fn start(
    main: fn(),
    _argc: isize,
    _argv: *const *const u8,
) -> isize {
    main();

    0
}

pub unsafe fn setup_bss() {
    let bss_start = &mut _bss_start as *mut u8;
    let bss_end = &mut _bss_end as *mut u8;
    let bss_len = bss_end as usize - bss_start as usize;
    let bss = slice::from_raw_parts_mut(bss_start, bss_len);
    for b in &mut bss.iter_mut() {
        *b = 0;
    }
}
