use core::sync::atomic::{AtomicBool,ATOMIC_BOOL_INIT,Ordering};

use volatile::Volatile;
use bit_field::BitField;

pub enum Clock {
    PortA,
    PortB,
    PortC,
    PortD,
    PortE,
    Uart0,
    Uart1,
    Uart2,
}

#[repr(C,packed)]
struct SimRegs {
    sopt1:      Volatile<u32>,
    sopt1_cfg:  Volatile<u32>,
    _pad0:      [u32; 1023],
    sopt2:      Volatile<u32>,
    _pad1:      u32,
    sopt4:      Volatile<u32>,
    sopt5:      Volatile<u32>,
    _pad2:      u32,
    sopt7:      Volatile<u32>,
    _pad3:      [u32; 2],
    sdid:       Volatile<u32>,
    _pad4:      u32,
    scgc2:      Volatile<u32>,
    scgc3:      Volatile<u32>,
    scgc4:      Volatile<u32>,
    scgc5:      Volatile<u32>,
    scgc6:      Volatile<u32>,
    scgc7:      Volatile<u32>,
    clkdiv1:    Volatile<u32>,
    clkviv2:    Volatile<u32>,
    fcfg1:      Volatile<u32>,
    fcfg2:      Volatile<u32>,
    uidh:       Volatile<u32>,
    uidmh:      Volatile<u32>,
    uidml:      Volatile<u32>,
    uidl:       Volatile<u32>
}

pub struct Sim {
    reg: &'static mut SimRegs
}

static SIM_INIT: AtomicBool = ATOMIC_BOOL_INIT;

impl Sim {
    pub fn new() -> Sim {
        let was_init = SIM_INIT.swap(true, Ordering::Relaxed);
        if was_init {
            panic!("Cannot initialize SIM: It's already active");
        }
        let reg = unsafe { &mut *(0x40047000 as *mut SimRegs) };
        Sim { reg: reg }
    }

    pub fn enable_clock(&mut self, clock: Clock) {
        match clock {
            Clock::PortA => {
                self.reg.scgc5.update(|scgc| {
                    scgc.set_bit(9, true);
                });
            }
            Clock::PortB => {
                self.reg.scgc5.update(|scgc| {
                    scgc.set_bit(10, true);
                });
            }
            Clock::PortC => {
                self.reg.scgc5.update(|scgc| {
                    scgc.set_bit(11, true);
                });
            }
            Clock::PortD => {
                self.reg.scgc5.update(|scgc| {
                    scgc.set_bit(12, true);
                });
            }
            Clock::PortE => {
                self.reg.scgc5.update(|scgc| {
                    scgc.set_bit(13, true);
                });
            }
            Clock::Uart0 => {
                self.reg.scgc4.update(|scgc| {
                    scgc.set_bit(10, true);
                });
            }
            Clock::Uart1 => {
                self.reg.scgc4.update(|scgc| {
                    scgc.set_bit(11, true);
                });
            }
            Clock::Uart2 => {
                self.reg.scgc4.update(|scgc| {
                    scgc.set_bit(12, true);
                });
            }
        }
    }

    pub fn set_dividers(&mut self, core: u32, bus: u32, flash: u32) {
        let mut clkdiv: u32 = 0;
        clkdiv.set_bits(28..32, core - 1);
        clkdiv.set_bits(24..28, bus - 1);
        clkdiv.set_bits(16..20, flash - 1);
        self.reg.clkdiv1.write(clkdiv);
    }
}

impl Drop for Sim {
    fn drop(&mut self) {
        SIM_INIT.store(false, Ordering::Relaxed);
    }
}
