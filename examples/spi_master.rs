#![no_main]
#![no_std]
#![no_builtins]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt;
extern crate embedded_hal;
extern crate mkl26;
extern crate nb;

use core::fmt::Write;

use embedded_hal::spi::{FullDuplex, Mode, Phase, Polarity};

use mkl26::mcg::{Clock, Mcg, OscRange};
use mkl26::osc::Osc;
use mkl26::port::PortName;
use mkl26::sim::cop::Cop;
use mkl26::sim::{ClkSrc, Sim};
use mkl26::spi::{Divisor, OpMode, Prescale, SpiMaster};
use mkl26::uart;

#[cfg_attr(rustfmt, rustfmt_skip)]
#[link_section = ".flashconfig"]
#[no_mangle]
pub static _FLASHCONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xDE, 0xF9, 0xFF, 0xFF
];

pre_init!(disable_wdog);

unsafe fn disable_wdog() {
    Cop::new().init(None);
}

entry!(main);

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

    let port_b = sim.port(PortName::B);
    let port_c = sim.port(PortName::C);
    let port_d = sim.port(PortName::D);

    let mut led = port_c.pin(5).to_gpio();
    led.output();

    let rx = port_b.pin(16).to_uart_rx().ok();
    let tx = port_b.pin(17).to_uart_tx().ok();
    unsafe {
        sim.set_uart0_clksrc(ClkSrc::McgXLL);
    }
    let mut uart = sim
        .uart(0, rx, tx, uart::calc_clkdiv(115200, 24_000_000))
        .unwrap();

    let mosi = port_c.pin(6).to_spi_mosi().ok();
    let miso = port_c.pin(7).to_spi_miso().ok();
    let sck = port_d.pin(1).to_spi_sck().unwrap();
    let cs = port_c.pin(4).to_spi_cs().ok();
    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase:    Phase::CaptureOnFirstTransition,
    };
    let mut spi: SpiMaster<u8> = sim
        .spi_master(
            mosi,
            miso,
            sck,
            cs,
            (Prescale::Div3, Divisor::Div2), // 4 MHz
            OpMode::IMM,
            spi_mode,
            true,
        ).unwrap();

    led.high();

    write!(uart, "SPI Test\r\n").unwrap();

    'main: loop {
        if let Ok(b) = uart.read_byte() {
            while let Err(_) = uart.write_byte(b) {}

            loop {
                match spi.send(b) {
                    Err(nb::Error::WouldBlock) => {}
                    Err(_) => {
                        write!(uart, "\r\nSPI write error\r\n").unwrap();
                        continue 'main;
                    }
                    Ok(_) => break,
                }
            }

            loop {
                match spi.read() {
                    Err(nb::Error::WouldBlock) => {}
                    Err(_) => {
                        write!(uart, "\r\nSPI read error\r\n").unwrap();
                        break;
                    }
                    Ok(b) => {
                        write!(uart, "\r\nSPI returned: {}\r\n", b).unwrap();
                        break;
                    }
                }
            }
        }
    }
}

//TODO: change to use USB_Listen for the panic messages
#[panic_handler]
pub fn rust_begin_panic(_info: &core::panic::PanicInfo) -> ! {
    // Reset the MCU after we've printed our panic.
    /*
    let aircr = unsafe {
        &mut *(0xE000ED0C as *mut Volatile<u32>)
    };
    aircr.write(0x05FA0004);
    */
    loop {}
}

// the hard fault handler
exception!(HardFault, hard_fault);

fn hard_fault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    loop {}
}

// the default exception handler
exception!(*, default_handler);

fn default_handler(_irqn: i16) {}
