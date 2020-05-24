#![no_main]
#![no_std]
#![no_builtins]

use cortex_m::interrupt as isr;
use cortex_m_rt::{entry, pre_init};
use mkl26z4::{interrupt, Interrupt, NVIC};
use panic_halt as _;

use mkl26::mcg::{Clock, Mcg, OscRange};
use mkl26::osc::Osc;
use mkl26::pit::{Pit, Timer, TimerSelect};
use mkl26::port::{Gpio, Port, PortName};
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

static mut PORT_C: Option<Port<{PortName::C}>> = None;
static mut LED_PIN: Option<Gpio<'static, {PortName::C}, 5>> = None;

static mut PIT0: Option<Pit> = None;
static mut PIT0_TIMER0: Option<Timer> = None;

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

    unsafe {
        PORT_C = Some(sim.port::<{PortName::C}>());

        LED_PIN = Some(PORT_C.as_mut().unwrap().pin::<5>().to_gpio());
        LED_PIN.as_mut().unwrap().output();

        // Interrupt Timers
        // The PIT it clocked off of the bus clock
        // Bus clock of 24 MHz is a period of 41.666 ns.
        // Trigger every 1 ms.
        // So 1 ms / 41.66 ns = 2400.
        // Subtract one for hardware implementation jazz.
        PIT0 = sim.pit().ok();
        PIT0_TIMER0 = PIT0
            .as_mut()
            .unwrap()
            .timer(TimerSelect::Timer0, 5000000, true)
            .ok();
        isr::free(|_| {
            NVIC::unpend(Interrupt::PIT);
            NVIC::unmask(Interrupt::PIT);
        });

        LED_PIN.as_mut().unwrap().high();

        PIT0_TIMER0.as_mut().unwrap().enable();
    }

    loop {}
}

#[interrupt]
fn PIT() {
    unsafe {
        LED_PIN.as_mut().unwrap().toggle();
        PIT0_TIMER0.as_mut().unwrap().clear_interrupt();
    }
}
