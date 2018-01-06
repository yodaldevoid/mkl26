#![feature(lang_items,asm)]
#![no_std]
#![no_main]
#![no_builtins]

extern crate rusty_teensy;

use core::fmt::Write;

use rusty_teensy::read::Read;

use rusty_teensy::mcg::{Clock,Mcg,OscRange};
use rusty_teensy::osc::Osc;
use rusty_teensy::port::{Port,PortName};
use rusty_teensy::sim::Sim;
use rusty_teensy::uart::Uart;
use rusty_teensy::watchdog::Watchdog;

#[link_section = ".flashconfig"]
#[no_mangle]
pub static _FLASHCONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF
];

static mut PANIC_PORT: Option<Port> = None;
static mut PANIC_WRITER: Option<Uart<'static, 'static>> = None;

#[no_mangle]
pub extern fn main() {
    unsafe {
        Watchdog::new().disable();
    }

    // Enable the crystal oscillator with 0pf of capacitance
    let osc_token = Osc::new().enable(0);

    // Set our clocks:
    // core/system: 100Mhz
    // bus: 50MHz
    // flash: 25MHz
    let mut sim = Sim::new();
    sim.set_dividers(1, 2, 4);
    // We would also set the USB divider here if we wanted to use it.

    let mcg = Mcg::new();
    if let Clock::Fei(mut fei) = mcg.clock() {
        // Our 50MHz xtal is "very fast", and needs to be divided by 1280 to be
        // in the acceptable FLL range and 1536 to be a little safer.
        fei.enable_xtal(OscRange::VeryHigh, osc_token);
        let fbe = fei.use_external_bypass(1536);

        // PLL is 32/16 * 50MHz == 100MHz
        let pbe = fbe.enable_pll(32, 16);
        pbe.use_pll();
    } else {
        panic!("Somehow the clock wasn't in FEI mode");
    }

    //let port_a = sim.port(PortName::A);
    let port_b = sim.port(PortName::B);
    //let port_c = sim.port(PortName::C);
    //let port_d = sim.port(PortName::D);
    let port_e = sim.port(PortName::E);

    //let rx = port_a.pin(1).to_uart_rx().unwrap();
    //let tx = port_a.pin(2).to_uart_tx().unwrap();
    let rx = port_b.pin(16).to_uart_rx().unwrap();
    let tx = port_b.pin(17).to_uart_tx().unwrap();
    let mut uart = sim.uart(0, Some(rx), Some(tx), (651, 1), true, false).unwrap();
    write!(uart, "Hello World\r\n").unwrap();
    
    //let mut red = port_b.pin(22).to_gpio();
    //let mut blue = port_b.pin(21).to_gpio();
    let mut green = port_e.pin(26).to_gpio();
    //red.output();
    //blue.output();
    green.output();
    //red.high();
    //blue.high();
    green.low();

    loop {
        let mut buf = [0; 1];
        match uart.read(&mut buf) {
            Ok(n) if n > 0 => {
                write!(uart, "{}", buf[0] as char).unwrap();

                green.high();
                for _ in 0..1000000 { unsafe { asm!{"nop"} } }
                green.low();
                for _ in 0..1000000 { unsafe { asm!{"nop"} } }
            }
            _ => {}
        }
    }
}

//TODO: change to use USB_Listen for the panic messages
#[lang = "panic_fmt"]
#[no_mangle]
pub extern fn begin_panic(_msg: core::fmt::Arguments,
                          _file: &'static str,
                          _line: u32,
                          _col: u32) -> ! {
    if let Some(uart) = unsafe { PANIC_WRITER.as_mut() } {
        write!(uart, "panicked at '").unwrap();
        uart.write_fmt(_msg).unwrap();
        write!(uart, "', {}:{}\n", _file, _line).unwrap();
    }

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
