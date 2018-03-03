use volatile_register::RW;
use bit_field::BitField;

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

// TODO: fake atomic
static mut OSC_INIT: bool = false;

impl Osc {
    pub fn new() -> Osc {
        // TODO: fake atomic
        let was_init = unsafe { OSC_INIT };
        unsafe { OSC_INIT = true; }
        if was_init {
            panic!("Cannot initialize OSC: It's already active");
        }
        let reg = unsafe { &mut *(0x40065000 as *mut OscRegs) };
        Osc { reg: reg }
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
        // TODO: fake atomic
        unsafe { OSC_INIT = false; }
    }
}

impl OscToken {
    pub(crate) fn new() -> OscToken {
        OscToken { _private: () }
    }
}
