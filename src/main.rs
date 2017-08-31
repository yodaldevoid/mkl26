#![feature(lang_items,asm)]
#![no_std]
#![no_main]
#![no_builtins]

extern crate volatile;
extern crate bit_field;

mod mcg;
mod osc;
mod port;
mod sim;
mod uart;
mod watchdog;

use core::slice;
use core::fmt::Write;

extern {
    static mut _bss_start: u8;
    static mut _bss_end: u8;

    fn _stack_top();
}

#[link_section = ".vectors"]
#[no_mangle]
pub static _VECTORS: [unsafe extern fn(); 2] = [
    _stack_top,
    main,
];

#[link_section = ".flashconfig"]
#[no_mangle]
pub static _FLASHCONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xDE, 0xF9, 0xFF, 0xFF
];

#[no_mangle]
pub extern fn main() {
    let (wdog, sim, pin) = unsafe {
        (watchdog::Watchdog::new(),
         sim::Sim::new(),
         port::Port::new(port::PortName::C).pin(5))
    };

    wdog.disable();
    unsafe { setup_bss() };
    // Enable the crystal oscillator with 10pf of capacitance
    let osc_token = osc::Osc::new().enable(10);

    // Set our clocks:
    // core: 72Mhz
    // peripheral: 36MHz
    // flash: 24MHz
    sim.set_dividers(1, 2, 3);
    // We would also set the USB divider here if we wanted to use it.

    let mcg = mcg::Mcg::new();
    if let mcg::Clock::Fei(mut fei) = mcg.clock() {
        // Our 16MHz xtal is "very fast", and needs to be divided
        // by 512 to be in the acceptable FLL range.
        fei.enable_xtal(mcg::OscRange::VeryHigh, osc_token);
        let fbe = fei.use_external(512);

        // PLL is 27/6 * xtal == 72MHz
        let pbe = fbe.enable_pll(27, 6);
        pbe.use_pll();
    } else {
        panic!("Somehow the clock wasn't in FEI mode");
    }

    sim.enable_clock(sim::Clock::PortB);
    sim.enable_clock(sim::Clock::Uart0);
    let uart = unsafe {
        let rx = port::Port::new(port::PortName::B).pin(16).to_uart_rx().unwrap();
        let tx = port::Port::new(port::PortName::B).pin(17).to_uart_tx().unwrap();
        uart::Uart::new(0, Some(rx), Some(tx), (468, 24)).unwrap()
    };

    writeln!(uart, "Hello World").unwrap();

    sim.enable_clock(sim::Clock::PortC);
    let mut gpio = pin.to_gpio();
    gpio.output();
    gpio.high();

    loop {}
}

#[lang = "panic_fmt"]
#[no_mangle]
pub extern fn rust_begin_unwind(_msg: core::fmt::Arguments,
                                _file: &'static str,
                                _line: u32) -> ! {
    loop {};
}

#[lang = "eh_personality"]
pub extern fn eh_personality() {}

unsafe fn setup_bss() {
    let bss_start = &mut _bss_start as *mut u8;
    let bss_end = &mut _bss_end as *mut u8;
    let bss_len = bss_end as usize - bss_start as usize;
    let bss = slice::from_raw_parts_mut(bss_start, bss_len);
    for b in &mut bss.iter_mut() {
        *b = 0;
    }
}
