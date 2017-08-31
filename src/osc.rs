use volatile::Volatile;
use bit_field::BitField;

#[repr(C,packed)]
pub struct Osc {
    cr: Volatile<u8>
}

impl Osc {
    pub unsafe fn new() -> &'static mut Osc {
        &mut *(0x40065000 as *mut Osc)
    }

    pub fn enable(&mut self, capacitance: u8) {
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

        self.cr.write(cr);
    }

    pub fn disable(&mut self) {
        self.cr.update(|cr| {
            cr.set_bit(7, false);
        });
    }
}