use bit_field::BitField;
use volatile_register::RW;

use atomic::InterruptAtomic;

const OSC_ADDR: usize = 0x4006_5000;

#[repr(C,packed)]
struct OscRegs {
    cr: RW<u8>
}

pub struct Osc {
    reg: &'static mut OscRegs
}

pub struct OscToken {
    _private: ()
}

static OSC_INIT: InterruptAtomic<bool> = InterruptAtomic::new(false);

impl Osc {
    pub fn new() -> Osc {
        let was_init = OSC_INIT.swap(true);
        if was_init {
            panic!("Cannot initialize OSC: It's already active");
        }
        let reg = unsafe { &mut *(OSC_ADDR as *mut OscRegs) };
        Osc { reg }
    }

    pub fn enable(&mut self, capacitance: u8) -> OscToken {
        if capacitance % 2 == 1 || capacitance > 30 {
            panic!("Invalid crystal capacitance value: {}", capacitance);
        }

        let mut cr: u8 = 0;
        cr.set_bit(3, capacitance.get_bit(1));
        cr.set_bit(2, capacitance.get_bit(2));
        cr.set_bit(1, capacitance.get_bit(3));
        cr.set_bit(0, capacitance.get_bit(4));

        // enable the crystal oscillator
        cr.set_bit(7, true);

        unsafe { self.reg.cr.write(cr); }

        OscToken::new()
    }

    pub fn disable(&mut self, _token: OscToken) {
        unsafe {
            self.reg.cr.modify(|mut cr| {
                cr.set_bit(7, false);
                cr
            });
        }
    }
}

impl Drop for Osc {
    fn drop(&mut self) {
        OSC_INIT.store(false);
    }
}

impl OscToken {
    pub(crate) fn new() -> OscToken {
        OscToken { _private: () }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn osc_regs() {
        unsafe {
            let reg = & *(OSC_ADDR as *const OscRegs);
            assert_eq!(0x4006_5000 as *const RW<u8>, &reg.cr as *const RW<u8>, "cr");
        }
    }
}
