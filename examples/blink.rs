#![feature(lang_items)]
#![feature(used)]
#![no_std]
#![no_builtins]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt;
extern crate mkl26;

use cortex_m::asm;

use mkl26::mcg::{Clock,Mcg,OscRange};
use mkl26::osc::Osc;
use mkl26::port::PortName;
use mkl26::sim::Sim;
use mkl26::sim::cop::Cop;

#[link_section = ".flashconfig"]
#[no_mangle]
pub static _FLASHCONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xDE, 0xF9, 0xFF, 0xFF
];

init_array!(disable_wdog, {
    unsafe { Cop::new().init(None); }
});

fn main() {
    // Enable the crystal oscillator with 10pf of capacitance
    let osc_token = Osc::new().enable(10);

    // Set our clocks:
    // core/system: 48Mhz
    // bus: 24MHz
    // flash: 24MHz
    let mut sim = Sim::new();
    sim.set_dividers(1, 2);
    // We would also set the USB divider here if we wanted to use it.
    let mcg = Mcg::new();
    if let Clock::Fei(mut fei) = mcg.clock() {
        // Our 16MHz xtal is "very fast", and needs to be divided
        // by 512 to be in the acceptable FLL range.
        let ext_token = fei.enable_xtal(OscRange::VeryHigh, osc_token);
        let fbe = fei.use_external_bypass(512, ext_token);

        // PLL is 24/8 * xtal == 48MHz
        let pbe = fbe.enable_pll(24, 8, &mut sim);
        pbe.use_pll();
    } else {
        panic!("Somehow the clock wasn't in FEI mode");
    }

    let port_c = sim.port(PortName::C);

    let mut led = port_c.pin(5).to_gpio();
    led.output();

    loop {
        led.toggle();
        for _ in 0..3_000_000 { asm::nop() }
    }
}

//TODO: change to use USB_Listen for the panic messages
#[lang = "panic_fmt"]
#[no_mangle]
pub extern fn begin_panic(_msg: core::fmt::Arguments,
                          _file: &'static str,
                          _line: u32,
                          _col: u32) -> ! {
    // Reset the MCU after we've printed our panic.
    /*
    let aircr = unsafe {
        &mut *(0xE000ED0C as *mut Volatile<u32>)
    };
    aircr.write(0x05FA0004);
    */
    loop {}
}

#[lang = "eh_personality"]
pub extern fn eh_personality() {}
