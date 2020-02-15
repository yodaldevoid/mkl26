#![no_main]
#![no_std]
#![no_builtins]

use core::fmt::Write;

use cortex_m_rt::{entry, pre_init};
use panic_halt as _;

use mkl26::mcg::{Clock, Mcg, OscRange};
use mkl26::osc::Osc;
use mkl26::port::PortName;
use mkl26::sim::cop::Cop;
use mkl26::sim::{ClkSrc, Sim};
use mkl26::uart::{self, UartNum};

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

    let port_c = sim.port(PortName::C);
    let mut led = port_c.pin(5).to_gpio();
    led.output();

    let port_b = sim.port(PortName::B);
    let rx = port_b.pin(16).to_uart_rx().ok();
    let tx = port_b.pin(17).to_uart_tx().ok();
    unsafe {
        sim.set_uart0_clksrc(ClkSrc::McgXLL);
    }
    let mut uart = sim
        .uart(
            UartNum::UART0,
            rx,
            tx,
            uart::calc_clkdiv(115200, 24_000_000),
        )
        .unwrap();
    //unsafe { sim.set_uart0_clksrc(Uart0ClkSrc::OscER); }
    //let mut uart = sim.uart(0, rx, tx, uart::calc_clkdiv(115200, 16_000_000)).unwrap();

    //let rx = port_c.pin(3).to_uart_rx().ok();
    //let tx = port_c.pin(4).to_uart_tx().ok();
    //let mut uart = sim.uart(1, rx, tx, uart::calc_clkdiv(115200, 24_000_000)).unwrap();

    led.high();

    write!(uart, "This is a test\r\n").unwrap();

    loop {
        if let Ok(b) = uart.read_byte() {
            match b {
                b'\r' => {
                    while let Err(_) = uart.write_byte(b'\r') {}
                    while let Err(_) = uart.write_byte(b'\n') {}
                }
                _ => while let Err(_) = uart.write_byte(b) {},
            }
        }
    }
}
