use core::mem;

use bit_field::BitField;
use volatile_register::{RO, RW};

use crate::atomic::InterruptAtomic;
use crate::osc::OscToken;
use crate::sim::{PllFllSel, Sim};

const MCG_ADDR: usize = 0x4006_4000;

#[repr(C, packed)]
struct McgRegs {
    c1: RW<u8>,
    c2: RW<u8>,
    c3: RW<u8>,
    c4: RW<u8>,
    c5: RW<u8>,
    c6: RW<u8>,
    s: RO<u8>,
    _pad0: u8,
    sc: RW<u8>,
    _pad1: u8,
    atcvh: RW<u8>,
    atcvl: RW<u8>,
    c7: RW<u8>,
    c8: RW<u8>,
    c9: RW<u8>,
    c10: RW<u8>,
}

pub struct Mcg {
    reg: &'static mut McgRegs,
}

#[derive(Clone, Copy)]
pub enum OscRange {
    Low = 0,
    High = 1,
    VeryHigh = 2,
}

enum OscSource {
    LockedLoop = 0,
    Internal = 1,
    External = 2,
}

pub enum Clock {
    Fei(Fei),
    Fee(Fee),
    Fbi(Fbi),
    Fbe(Fbe),
    Pee(Pee),
    Pbe(Pbe),
    Blpi(Blpi),
    Blpe(Blpe),
    Stop(Stop),
}

static MCG_INIT: InterruptAtomic<bool> = InterruptAtomic::new(false);

impl Mcg {
    pub fn new() -> Mcg {
        let was_init = MCG_INIT.swap(true);
        if was_init {
            panic!("Cannot initialize MCG: It's already active");
        }
        let reg = unsafe { &mut *(MCG_ADDR as *mut McgRegs) };
        Mcg { reg }
    }

    //TODO: Stop
    pub fn clock(self) -> Clock {
        let source: OscSource = unsafe { mem::transmute(self.reg.c1.read().get_bits(6..8)) };
        let fll_internal = self.reg.c1.read().get_bit(2);
        let pll_enabled = self.reg.c6.read().get_bit(6);
        let low_power = self.reg.c2.read().get_bit(1);

        match (source, fll_internal, pll_enabled, low_power) {
            (OscSource::LockedLoop, true, false, _) => Clock::Fei(Fei { mcg: self }),
            (OscSource::LockedLoop, false, false, _) => Clock::Fee(Fee { mcg: self }),
            (OscSource::Internal, true, false, false) => Clock::Fbi(Fbi { mcg: self }),
            (OscSource::External, false, false, false) => Clock::Fbe(Fbe { mcg: self }),
            (OscSource::LockedLoop, false, true, _) => Clock::Pee(Pee { mcg: self }),
            (OscSource::External, false, true, false) => Clock::Pbe(Pbe { mcg: self }),
            (OscSource::Internal, true, false, true) => Clock::Blpi(Blpi { mcg: self }),
            (OscSource::External, false, _, true) => Clock::Blpe(Blpe { mcg: self }),
            _ => panic!("The current clock mode cannot be represented as a known struct"),
        }
    }
}

impl Drop for Mcg {
    fn drop(&mut self) {
        MCG_INIT.store(false);
    }
}

pub struct Fei {
    mcg: Mcg,
}

pub struct Fee {
    mcg: Mcg,
}

pub struct Fbi {
    mcg: Mcg,
}

pub struct Fbe {
    mcg: Mcg,
}

pub struct Pbe {
    mcg: Mcg,
}

pub struct Pee {
    mcg: Mcg,
}

pub struct Blpi {
    mcg: Mcg,
}

pub struct Blpe {
    mcg: Mcg,
}

pub struct Stop {
    _mcg: Mcg,
}

pub struct ExtToken {
    _private: (),
}

impl ExtToken {
    fn new() -> ExtToken {
        ExtToken { _private: () }
    }
}

impl Fei {
    pub fn enable_xtal(&mut self, range: OscRange, _token: OscToken) -> ExtToken {
        unsafe {
            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bits(4..6, range as u8); // frequency range
                c2.set_bit(2, true); // external osc
                c2
            });
        }

        while !self.mcg.reg.s.read().get_bit(1) {}

        ExtToken::new()
    }

    // TODO: Should take token, but token might be gone by the time we get here
    pub fn disable_xtal(&mut self) {
        unsafe {
            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bit(2, false); // internal osc
                c2
            });
        }

        while self.mcg.reg.s.read().get_bit(1) {}
    }

    pub fn use_external(self, divide: u32, _token: ExtToken) -> Fee {
        let osc = self.mcg.reg.c2.read().get_bits(4..6);
        let frdiv = if osc == OscRange::Low as u8 {
            match divide {
                1 => 0,
                2 => 1,
                4 => 2,
                8 => 3,
                16 => 4,
                32 => 5,
                64 => 6,
                128 => 7,
                _ => panic!("Invalid external clock divider: {}", divide),
            }
        } else {
            match divide {
                32 => 0,
                64 => 1,
                128 => 2,
                256 => 3,
                512 => 4,
                1024 => 5,
                1280 => 6,
                1536 => 7,
                _ => panic!("Invalid external clock divider: {}", divide),
            }
        };

        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(3..6, frdiv);
                c1.set_bit(2, false); // external clock
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // Wait for the FLL to be pointed at the crystal
        while self.mcg.reg.s.read().get_bit(4) {}

        Fee { mcg: self.mcg }
    }

    pub fn bypass(self) -> Fbi {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::Internal as u8);
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // Wait for our clock source to be the internal osc
        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::Internal as u8 {}

        Fbi { mcg: self.mcg }
    }

    pub fn use_external_bypass(self, divide: u32, _token: ExtToken) -> Fbe {
        let osc = self.mcg.reg.c2.read().get_bits(4..6);
        let frdiv = if osc == OscRange::Low as u8 {
            match divide {
                1 => 0,
                2 => 1,
                4 => 2,
                8 => 3,
                16 => 4,
                32 => 5,
                64 => 6,
                128 => 7,
                _ => panic!("Invalid external clock divider: {}", divide),
            }
        } else {
            match divide {
                32 => 0,
                64 => 1,
                128 => 2,
                256 => 3,
                512 => 4,
                1024 => 5,
                1280 => 6,
                1536 => 7,
                _ => panic!("Invalid external clock divider: {}", divide),
            }
        };

        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::External as u8);
                c1.set_bits(3..6, frdiv);
                c1.set_bit(2, false); // external clock
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // First: Wait for the FLL to be pointed at the crystal
        // Then: Wait for our clock source to be the crystal osc
        while self.mcg.reg.s.read().get_bit(4) {}
        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::External as u8 {}

        Fbe { mcg: self.mcg }
    }
}

impl Fee {
    pub fn use_internal(self) -> Fei {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bit(2, true); // internal clock
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // Wait for the FLL to be pointed at the internal clock
        while !self.mcg.reg.s.read().get_bit(4) {}

        Fei { mcg: self.mcg }
    }

    pub fn bypass(self) -> Fbe {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::External as u8);
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // Wait for our clock source to be the crystal osc
        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::External as u8 {}

        Fbe { mcg: self.mcg }
    }

    pub fn use_internal_bypass(self) -> Fbi {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::Internal as u8);
                c1.set_bit(2, true); // internal clock
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // First: Wait for the FLL to be pointed at the internal clock
        // Then: Wait for our clock source to be the internal clock
        while !self.mcg.reg.s.read().get_bit(4) {}
        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::Internal as u8 {}

        Fbi { mcg: self.mcg }
    }
}

impl Fbi {
    pub fn enable_xtal(&mut self, range: OscRange, _token: OscToken) -> ExtToken {
        unsafe {
            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bits(4..6, range as u8); // frequency range
                c2.set_bit(2, true); // external osc
                c2
            });
        }

        while !self.mcg.reg.s.read().get_bit(1) {}

        ExtToken::new()
    }

    // TODO: Should take token, but token might be gone by the time we get here
    pub fn disable_xtal(&mut self) {
        unsafe {
            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bit(2, false); // internal osc
                c2
            });
        }

        while self.mcg.reg.s.read().get_bit(1) {}
    }

    pub fn use_external(self, divide: u32, _token: ExtToken) -> Fbe {
        let osc = self.mcg.reg.c2.read().get_bits(4..6);
        let frdiv = if osc == OscRange::Low as u8 {
            match divide {
                1 => 0,
                2 => 1,
                4 => 2,
                8 => 3,
                16 => 4,
                32 => 5,
                64 => 6,
                128 => 7,
                _ => panic!("Invalid external clock divider: {}", divide),
            }
        } else {
            match divide {
                32 => 0,
                64 => 1,
                128 => 2,
                256 => 3,
                512 => 4,
                1024 => 5,
                1280 => 6,
                1536 => 7,
                _ => panic!("Invalid external clock divider: {}", divide),
            }
        };

        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(3..6, frdiv);
                c1.set_bit(2, false); // external clock
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // Wait for the FLL to be pointed at the crystal
        while self.mcg.reg.s.read().get_bit(4) {}

        Fbe { mcg: self.mcg }
    }

    pub fn use_fll(self) -> Fei {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::LockedLoop as u8);
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // Wait for our clock source to be the FLL
        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::LockedLoop as u8 {}

        Fei { mcg: self.mcg }
    }

    pub fn use_fll_external(self, divide: u32, _token: ExtToken) -> Fee {
        let osc = self.mcg.reg.c2.read().get_bits(4..6);
        let frdiv = if osc == OscRange::Low as u8 {
            match divide {
                1 => 0,
                2 => 1,
                4 => 2,
                8 => 3,
                16 => 4,
                32 => 5,
                64 => 6,
                128 => 7,
                _ => panic!("Invalid external clock divider: {}", divide),
            }
        } else {
            match divide {
                32 => 0,
                64 => 1,
                128 => 2,
                256 => 3,
                512 => 4,
                1024 => 5,
                1280 => 6,
                1536 => 7,
                _ => panic!("Invalid external clock divider: {}", divide),
            }
        };

        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::LockedLoop as u8);
                c1.set_bits(3..6, frdiv);
                c1.set_bit(2, false); // external clock
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // First: Wait for the FLL to be pointed at the crystal
        // Then: Wait for our clock source to be the external osc
        while self.mcg.reg.s.read().get_bit(4) {}
        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::LockedLoop as u8 {}

        Fee { mcg: self.mcg }
    }

    pub fn low_power(self) -> Blpi {
        unsafe {
            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bit(1, true); // low power
                c2
            });
        }

        Blpi { mcg: self.mcg }
    }
}

impl Fbe {
    pub fn use_internal(self) -> Fbi {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bit(2, true); // internal clock
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // Wait for the FLL to be pointed at the internal clock
        while !self.mcg.reg.s.read().get_bit(4) {}

        Fbi { mcg: self.mcg }
    }

    pub fn use_fll(self) -> Fee {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::LockedLoop as u8);
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // Wait for our clock source to be the FLL
        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::LockedLoop as u8 {}

        Fee { mcg: self.mcg }
    }

    pub fn use_fll_internal(self) -> Fei {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::LockedLoop as u8);
                c1.set_bit(2, true); // internal clock
                c1
            });
        }

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // First: Wait for the FLL to be pointed at the internal clock
        // Then: Wait for our clock source to be the FLL
        while !self.mcg.reg.s.read().get_bit(4) {}
        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::LockedLoop as u8 {}

        Fei { mcg: self.mcg }
    }

    pub fn enable_pll(self, numerator: u8, denominator: u8, sim: &mut Sim) -> Pbe {
        if numerator < 24 || numerator > 55 {
            panic!("Invalid PLL VCO divide factor: {}", numerator);
        }

        if denominator < 1 || denominator > 25 {
            panic!("Invalid PLL reference divide factor: {}", denominator);
        }

        unsafe {
            self.mcg.reg.c5.modify(|mut c5| {
                c5.set_bits(0..5, denominator - 1);
                c5
            });

            self.mcg.reg.c6.modify(|mut c6| {
                c6.set_bits(0..5, numerator - 24);
                c6.set_bit(6, true);
                c6
            });
        }

        // Wait for PLL to be enabled
        while !self.mcg.reg.s.read().get_bit(5) {}
        // Wait for the PLL to be "locked" and stable
        while !self.mcg.reg.s.read().get_bit(6) {}

        unsafe {
            sim.select_pll_fll(PllFllSel::PllDiv2);
        }

        Pbe { mcg: self.mcg }
    }

    pub fn low_power(self) -> Blpe {
        unsafe {
            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bit(1, true); // low power
                c2
            });
        }

        Blpe { mcg: self.mcg }
    }
}

impl Pbe {
    pub fn use_pll(self) -> Pee {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::LockedLoop as u8);
                c1
            });
        }

        // mcg.c1 and mcg.s have slightly different behaviors.  In c1,
        // we use one value to indicate "Use whichever LL is
        // enabled". In s, it is differentiated between the FLL at 0,
        // and the PLL at 3. Instead of adding a value to OscSource
        // which would be invalid to set, we just check for the known
        // value "3" here.
        while self.mcg.reg.s.read().get_bits(2..4) != 3 {}

        Pee { mcg: self.mcg }
    }

    pub fn enable_fll(self, sim: &mut Sim) -> Fbe {
        unsafe {
            self.mcg.reg.c6.modify(|mut c6| {
                c6.set_bit(6, false);
                c6
            });
        }

        // Wait for FLL to be enabled
        while self.mcg.reg.s.read().get_bit(5) {}

        unsafe {
            sim.select_pll_fll(PllFllSel::Fll);
        }

        Fbe { mcg: self.mcg }
    }

    pub fn low_power(self) -> Blpe {
        unsafe {
            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bit(1, true); // low power
                c2
            });
        }

        Blpe { mcg: self.mcg }
    }
}

impl Pee {
    pub fn bypass(self) -> Pbe {
        unsafe {
            self.mcg.reg.c1.modify(|mut c1| {
                c1.set_bits(6..8, OscSource::External as u8);
                c1
            });
        }

        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::External as u8 {}

        Pbe { mcg: self.mcg }
    }
}

impl Blpi {
    pub fn enable_fll(self) -> Fbi {
        unsafe {
            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bit(1, false); // disable low power
                c2
            });
        }

        // Wait for FLL to be enabled
        while self.mcg.reg.s.read().get_bit(5) {}

        Fbi { mcg: self.mcg }
    }
}

impl Blpe {
    pub fn enable_fll(self, sim: &mut Sim) -> Fbe {
        unsafe {
            self.mcg.reg.c6.modify(|mut c6| {
                c6.set_bit(6, false);
                c6
            });

            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bit(1, false); // disable low power
                c2
            });
        }

        // Wait for FLL to be enabled
        while self.mcg.reg.s.read().get_bit(5) {}

        unsafe {
            sim.select_pll_fll(PllFllSel::Fll);
        }

        Fbe { mcg: self.mcg }
    }

    pub fn enable_pll(self, numerator: u8, denominator: u8, sim: &mut Sim) -> Pbe {
        if numerator < 24 || numerator > 55 {
            panic!("Invalid PLL VCO divide factor: {}", numerator);
        }

        if denominator < 1 || denominator > 25 {
            panic!("Invalid PLL reference divide factor: {}", denominator);
        }

        unsafe {
            self.mcg.reg.c5.modify(|mut c5| {
                c5.set_bits(0..5, denominator - 1);
                c5
            });

            self.mcg.reg.c6.modify(|mut c6| {
                c6.set_bits(0..5, numerator - 24);
                c6.set_bit(6, true);
                c6
            });

            self.mcg.reg.c2.modify(|mut c2| {
                c2.set_bit(1, false); // disable low power
                c2
            });
        }

        // Wait for PLL to be enabled
        while !self.mcg.reg.s.read().get_bit(5) {}
        // Wait for the PLL to be "locked" and stable
        while !self.mcg.reg.s.read().get_bit(6) {}

        unsafe {
            sim.select_pll_fll(PllFllSel::PllDiv2);
        }

        Pbe { mcg: self.mcg }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn mcg_regs() {
        unsafe {
            let reg = &*(MCG_ADDR as *const McgRegs);
            assert_eq!(0x4006_4000 as *const RW<u8>, &reg.c1    as *const RW<u8>, "c1");
            assert_eq!(0x4006_4001 as *const RW<u8>, &reg.c2    as *const RW<u8>, "c2");
            assert_eq!(0x4006_4002 as *const RW<u8>, &reg.c3    as *const RW<u8>, "c3");
            assert_eq!(0x4006_4003 as *const RW<u8>, &reg.c4    as *const RW<u8>, "c4");
            assert_eq!(0x4006_4004 as *const RW<u8>, &reg.c5    as *const RW<u8>, "c5");
            assert_eq!(0x4006_4005 as *const RW<u8>, &reg.c6    as *const RW<u8>, "c6");
            assert_eq!(0x4006_4006 as *const RO<u8>, &reg.s     as *const RO<u8>, "s");
            assert_eq!(0x4006_4008 as *const RW<u8>, &reg.sc    as *const RW<u8>, "sc");
            assert_eq!(0x4006_400A as *const RW<u8>, &reg.atcvh as *const RW<u8>, "atcvh");
            assert_eq!(0x4006_400B as *const RW<u8>, &reg.atcvl as *const RW<u8>, "atcvl");
            assert_eq!(0x4006_400C as *const RW<u8>, &reg.c7    as *const RW<u8>, "c7");
            assert_eq!(0x4006_400D as *const RW<u8>, &reg.c8    as *const RW<u8>, "c8");
            assert_eq!(0x4006_400E as *const RW<u8>, &reg.c9    as *const RW<u8>, "c9");
            assert_eq!(0x4006_400F as *const RW<u8>, &reg.c10   as *const RW<u8>, "c10");
        }
    }
}
