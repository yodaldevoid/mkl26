#![feature(lang_items,asm)]
#![no_std]
#![no_builtins]

extern crate rusty_teensy;

use core::fmt::Write;

use rusty_teensy::read::Read;
use rusty_teensy::mcg::{Clock,Mcg,OscRange};
use rusty_teensy::osc::Osc;
use rusty_teensy::port::{Port,PortName};
use rusty_teensy::sim::Sim;
use rusty_teensy::watchdog::Watchdog;
use rusty_teensy::uart::Uart;

#[link_section = ".flashconfig"]
#[no_mangle]
pub static _FLASHCONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xDE, 0xF9, 0xFF, 0xFF
];

static mut PANIC_PORT: Option<Port> = None;
static mut PANIC_WRITER: Option<Uart<'static, 'static>> = None;

macro_rules! print {
    ($($arg:tt)*) => (
        if let Some(writer) = unsafe { PANIC_WRITER.as_mut() } {
            write!(writer, $($arg)*)
        } else {
            Err(core::fmt::Error)
        }
    );
}

macro_rules! println {
    () => (print!("\r\n"));
    ($fmt:expr) => (print!(concat!($fmt, "\r\n")));
    ($fmt:expr, $($arg:tt)*) => (print!(concat!($fmt, "\r\n"), $($arg)*));
}

fn main() {
    unsafe {
        Watchdog::new().disable();
    }

    // Enable the crystal oscillator with 10pf of capacitance
    let osc_token = Osc::new().enable(10);

    // Set our clocks:
    // core/system: 72Mhz
    // bus: 36MHz
    // flash: 24MHz
    let mut sim = Sim::new();
    sim.set_dividers(1, 2, 3);
    // We would also set the USB divider here if we wanted to use it.

    let mcg = Mcg::new();
    if let Clock::Fei(mut fei) = mcg.clock() {
        // Our 16MHz xtal is "very fast", and needs to be divided
        // by 512 to be in the acceptable FLL range.
        let ext_token = fei.enable_xtal(OscRange::VeryHigh, osc_token);
        let fbe = fei.use_external_bypass(512, ext_token);

        // PLL is 27/6 * xtal == 72MHz
        let pbe = fbe.enable_pll(27, 6);
        pbe.use_pll();
    } else {
        panic!("Somehow the clock wasn't in FEI mode");
    }

    //let port_a = sim.port(PortName::A);
    //let port_b = sim.port(PortName::B);
    unsafe { PANIC_PORT = Some(sim.port(PortName::C)); }
    //let port_d = sim.port(PortName::D);
    //let port_e = sim.port(PortName::E);

    unsafe {
        let rx = PANIC_PORT.as_mut().unwrap().pin(3).to_uart_rx().unwrap();
        let tx = PANIC_PORT.as_mut().unwrap().pin(4).to_uart_tx().unwrap();

        PANIC_WRITER = Some(sim.uart(1, rx, tx, (468, 24), true, true).unwrap());
    }

    println!("\r\nAdc Test").unwrap();

    let adc_pin = unsafe { PANIC_PORT.as_mut().unwrap().pin(0).to_adc().unwrap() };
    let mut adc = sim.adc(0, 14, 12, 8, adc_pin).unwrap(); // A1
    if adc.calibrate().is_err() {
        println!("ADC calibration failed").unwrap();
    }

    let mut led = unsafe { PANIC_PORT.as_mut().unwrap().pin(5).to_gpio() };
    led.output();
    led.high();

    loop {
        let mut buf = [0; 1];
        let uart = unsafe { PANIC_WRITER.as_mut().unwrap() };
        match uart.read(&mut buf) {
            Ok(n) if n > 0 => {
                led.low();
                adc.start_conv();
                while !adc.is_conv_done() {}
                led.high();
                println!("ADC value {0}", adc.read()).unwrap();
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
