#![no_main]
#![no_std]
#![no_builtins]

use core::fmt::Write;

use cortex_m::asm;
use cortex_m_rt::{entry, exception, pre_init};

use mkl26::mcg::{Clock, Mcg, OscRange};
use mkl26::osc::Osc;
use mkl26::port::PortName;
use mkl26::sim::cop::Cop;
use mkl26::sim::{ClkSrc, Sim};
use mkl26::tpm::{ChannelMode, ChannelSelect, PwmDirection};
use mkl26::tpm::{ClockMode, Prescale, PwmSelect, TpmNum};
use mkl26::uart;

#[cfg_attr(rustfmt, rustfmt_skip)]
#[link_section = ".flashconfig"]
#[no_mangle]
pub static _FLASHCONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xDE, 0xF9, 0xFF, 0xFF
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

    let pwm_pin = port_d.pin(4).to_tpm().ok();

    // sets register value in sopt2 to source TPM to PLL/2
    unsafe {
        sim.set_tpm_clksrc(ClkSrc::McgXLL);
    }

    let tpm0 = sim
        .tpm_periodic(
            TpmNum::TPM0,
            PwmSelect::Up,
            ClockMode::EveryClock,
            Prescale::Div8,
            false,
            0x6000,
        ).unwrap();

    let mut tpm0_ch4 = tpm0
        .channel(
            ChannelSelect::Ch4,
            ChannelMode::EdgePwm(PwmDirection::HighTrue),
            false,
            0x1E00,
            pwm_pin,
        ).unwrap();

    led.high();

    write!(uart, "PWM Test\r\n").unwrap();

    loop {
        asm::delay(100_000_000);
        led.low();
        tpm0_ch4.set_value(0x1000);
        asm::delay(100_000_000);
        led.high();
        tpm0_ch4.set_value(0x500);
        asm::delay(100_000_000);
        led.low();
        tpm0_ch4.set_value(0x1E00);
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
#[exception]
fn HardFault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    loop {}
}

// the default exception handler
#[exception]
fn DefaultHandler(_irqn: i16) {}
