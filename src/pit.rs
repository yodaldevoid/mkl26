use bit_field::BitField;
use cortex_m::interrupt;
use volatile_register::{RO, RW};

use sim::ClockGate;

const PIT_ADDR: usize = 0x4003_7000;

#[repr(C, packed)]
struct PitRegs {
    mcr: RW<u32>,

    _pad0: [u8; 220],

    ltmr64h: RO<u32>,
    ltmr64l: RO<u32>,

    _pad1: [u8; 24],

    timer: [TimerRegs; 2],
}

#[repr(C, packed)]
struct TimerRegs {
    ldvaln: RW<u32>,
    cvaln:  RO<u32>,
    tctrln: RW<u32>,
    tflgn:  RW<u32>,
}

#[derive(Clone, Copy, Debug)]
pub enum TimerSelect {
    Timer0 = 0,
    Timer1 = 1,
}

pub struct Pit {
    reg:   &'static mut PitRegs,
    _gate: ClockGate,
}

impl Pit {
    pub unsafe fn new(gate: ClockGate) -> Pit {
        let reg = &mut *(PIT_ADDR as *mut PitRegs);

        // Enable the module and allow timers to run in debug mode.
        reg.mcr.write(0);

        Pit { reg, _gate: gate }
    }

    /// The timer returned is not enabled. `Timer::enable` must be called for
    /// the timer to start.
    pub fn timer<'a>(
        &'a self,
        timer: TimerSelect,
        start: u32,
        interrupt: bool,
    ) -> Result<Timer<'a>, ()> {
        unsafe {
            let timer_reg = &self.reg.timer[timer as usize];
            interrupt::free(|_| Timer::new(timer_reg, start, interrupt))
        }
    }
}

// TODO: timer chaining
pub struct Timer<'a> {
    reg: &'a TimerRegs,
}

// TODO: set up the interrupts for the user
impl<'a> Timer<'a> {
    unsafe fn new(reg: &'a TimerRegs, start: u32, interrupt: bool) -> Result<Timer<'a>, ()> {
        if reg.tctrln.read().get_bit(0) {
            return Err(());
        }

        reg.ldvaln.write(start);

        let mut tctrl = 0;
        tctrl.set_bit(1, interrupt); // Enable/disable interrupts
        reg.tctrln.write(tctrl);

        Ok(Timer { reg })
    }

    pub fn enable(&mut self) {
        unsafe {
            self.reg.tctrln.modify(|mut tctrl| {
                tctrl.set_bit(0, true);
                tctrl
            });
        }
    }

    pub fn disable(&mut self) {
        unsafe {
            self.reg.tctrln.modify(|mut tctrl| {
                tctrl.set_bit(0, false);
                tctrl
            });
        }
    }

    pub fn restart(&mut self) {
        unsafe {
            self.reg.tctrln.modify(|mut tctrl| {
                tctrl.set_bit(0, false);
                tctrl
            });

            self.reg.tctrln.modify(|mut tctrl| {
                tctrl.set_bit(0, true);
                tctrl
            });
        }
    }

    pub fn set_start(&mut self, start: u32) {
        unsafe {
            self.reg.ldvaln.write(start);
        }
    }

    pub fn get_value(&self) -> u32 {
        unsafe { self.reg.cvaln.read() }
    }

    pub fn clear_interrupt(&mut self) {
        unsafe {
            self.reg.tflgn.write(1);
        }
    }
}

impl<'a> Drop for Timer<'a> {
    fn drop(&mut self) {
        unsafe {
            self.reg.tctrln.write(0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn pit_test() {
        unsafe {
            let reg = & *(PIT_ADDR as *const PitRegs);
            assert_eq!(0x4003_7000 as *const RW<u32>, &reg.mcr              as *const RW<u32>, "mcr");
            assert_eq!(0x4003_70E0 as *const RO<u32>, &reg.ltmr64h          as *const RO<u32>, "ltmr64h");
            assert_eq!(0x4003_70E4 as *const RO<u32>, &reg.ltmr64l          as *const RO<u32>, "ltmr64l");
            assert_eq!(0x4003_7100 as *const RW<u32>, &reg.timer[0].ldvaln  as *const RW<u32>, "ldval0");
            assert_eq!(0x4003_7104 as *const RO<u32>, &reg.timer[0].cvaln   as *const RO<u32>, "cval0");
            assert_eq!(0x4003_7108 as *const RW<u32>, &reg.timer[0].tctrln  as *const RW<u32>, "tctrl0");
            assert_eq!(0x4003_710C as *const RW<u32>, &reg.timer[0].tflgn   as *const RW<u32>, "tflg0");
            assert_eq!(0x4003_7110 as *const RW<u32>, &reg.timer[1].ldvaln  as *const RW<u32>, "ldval1");
            assert_eq!(0x4003_7114 as *const RO<u32>, &reg.timer[1].cvaln   as *const RO<u32>, "cval1");
            assert_eq!(0x4003_7118 as *const RW<u32>, &reg.timer[1].tctrln  as *const RW<u32>, "tctrl1");
            assert_eq!(0x4003_711C as *const RW<u32>, &reg.timer[1].tflgn   as *const RW<u32>, "tflg1");
        }
    }
}
