#![no_main]
#![no_std]
#![no_builtins]

#![feature(const_generics)]

use cortex_m::asm;
use cortex_m_rt::{entry, pre_init};
use panic_halt as _;

use mkl26::mcg::{Clock, Mcg, OscRange};
use mkl26::osc::Osc;
use mkl26::port::PortName;
use mkl26::sim::cop::Cop;
use mkl26::sim::Sim;

#[cfg_attr(rustfmt, rustfmt_skip)]
#[link_section = ".flashconfig"]
#[no_mangle]
pub static _FLASHCONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xDE, 0xF9, 0xFF, 0xFF,
];

#[pre_init]
unsafe fn disable_wdog() {
    Cop::new().init(None);
}

#[entry]
fn main() -> ! {
    // Enable the crystal oscillator with 10 pf of capacitance
    let osc_token = Osc::new().enable(10);

    // Set our clocks:
    // core/system: 48 Mhz
    // bus: 24 MHz
    // flash: 24 MHz
    let mut sim = Sim::new();
    sim.set_dividers(1, 2);
    let mcg = Mcg::new();
    if let Clock::Fei(mut fei) = mcg.clock() {
        // Our 16 MHz xtal is "very fast", and needs to be divided
        // by 512 to be in the acceptable FLL range.
        let ext_token = fei.enable_xtal(OscRange::VeryHigh, osc_token);
        let fbe = fei.use_external_bypass(512, ext_token);

        // PLL is 24/8 * xtal == 48 MHz
        let pbe = fbe.enable_pll(24, 8, &mut sim);
        pbe.use_pll();
    } else {
        panic!("Somehow the clock wasn't in FEI mode");
    }

    let port_c = sim.port::<{PortName::C}>();

    let mut led = port_c.pin::<5>().to_gpio();
    led.output();

    loop {
        led.toggle();
        asm::delay(3_000_000);
    }
}
