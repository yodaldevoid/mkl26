use core::sync::atomic::{AtomicBool,ATOMIC_BOOL_INIT,Ordering};

use volatile::Volatile;
use bit_field::BitField;

#[repr(C,packed)]
struct OscRegs {
    cr: Volatile<u8>
}

pub struct Osc {
    reg: &'static mut OscRegs
}

pub struct OscToken {
    _private: ()
}

static OSC_INIT: AtomicBool = ATOMIC_BOOL_INIT;

impl Osc {
    pub fn new() -> Osc {
        let was_init = OSC_INIT.swap(true, Ordering::Relaxed);
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

        self.reg.cr.write(cr);

        OscToken::new()
    }

    pub fn disable(&mut self, _token: OscToken) {
        self.reg.cr.update(|cr| {
            cr.set_bit(7, false);
        });
    }
}

impl Drop for Osc {
    fn drop(&mut self) {
        OSC_INIT.store(false, Ordering::Relaxed);
    }
}

impl OscToken {
    pub(crate) fn new() -> OscToken {
        OscToken { _private: () }
    }
}
