#![feature(lang_items,asm)]
#![no_std]
#![no_builtins]

extern crate volatile;
extern crate bit_field;

use core::slice;

pub mod mcg;
pub mod osc;
pub mod port;
pub mod sim;
pub mod uart;
pub mod watchdog;

extern {
    static mut _bss_start: u8;
    static mut _bss_end: u8;

    pub fn _stack_top();
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

#[no_mangle]
pub extern fn _fault() {
    unreachable!()
}

#[no_mangle]
pub extern fn default_isr() {
    loop {}
}
