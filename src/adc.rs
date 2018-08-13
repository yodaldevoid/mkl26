use volatile_register::{RO,RW};
use bit_field::BitField;

use port::{AdcPin,AdcDiffPPin,AdcDiffMPin,PortName};
use sim::ClockGate;

const ADC0_ADDR: usize = 0x4003_B000;

#[repr(C,packed)]
struct AdcRegs {
    sc1a:   RW<u32>,
    sc1b:   RW<u32>,
    cfg1:   RW<u32>,
    cfg2:   RW<u32>,
    ra:     RO<u32>,
    rb:     RO<u32>,
    cv1:    RW<u32>,
    cv2:    RW<u32>,
    sc2:    RW<u32>,
    sc3:    RW<u32>,
    ofs:    RW<u32>,
    pg:     RW<u32>,
    mg:     RW<u32>,
    clpd:   RW<u32>,
    clps:   RW<u32>,
    clp4:   RW<u32>,
    clp3:   RW<u32>,
    clp2:   RW<u32>,
    clp1:   RW<u32>,
    clp0:   RW<u32>,

    _pad0: u32,

    clmd:   RW<u32>,
    clms:   RW<u32>,
    clm4:   RW<u32>,
    clm3:   RW<u32>,
    clm2:   RW<u32>,
    clm1:   RW<u32>,
    clm0:   RW<u32>,
}

pub struct Adc<'a> {
    reg: &'static mut AdcRegs,
    _pin: Option<AdcPin<'a>>,
    _gate: ClockGate
}

pub struct AdcDiff<'a,'b> {
    reg: &'static mut AdcRegs,
    _pos: Option<AdcDiffPPin<'a>>,
    _neg: Option<AdcDiffMPin<'b>>,
    _gate: ClockGate
}

pub enum Resolution {
    Bits8_9 = 0,
    Bits10_11 = 2,
    Bits12_13 = 1,
    Bits16 = 3,
}

pub enum Divisor {
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
}

pub enum VoltageRef {
    Default = 0,
    Alternative = 1,
}

enum AdcMux {
    A,
    B,
}

impl AdcMux {
    fn value(&self) -> bool {
        match *self {
            AdcMux::A => false,
            AdcMux::B => true,
        }
    }
}

/// V_REFH and V_REFL are broken out as external pins 48-pin and higher devices.
/// V_REFH and V_REFL are connected to VDDA and VSSA respectively on 32-pin
/// devices.
///
/// V_ALTH and V_ALTL are connected to VDDA and VSSA respectively.
impl<'a> Adc<'a> {
    pub unsafe fn new(id: u8,
                      ch: u8,
                      resolution: Resolution,
                      clkdiv: Divisor,
                      vref: VoltageRef,
                      pin: Option<AdcPin<'a>>,
                      gate: ClockGate)
                      -> Result<Adc<'a>, ()> {
        let pin_info = pin.as_ref().map(|p| (p.port_name(), p.pin()));
        let mux = match (id, ch, pin_info) {
            // ADC0
            (0, 0,  Some((PortName::E, 20))) => AdcMux::A, // e20:0-S0,0
            (0, 1,  Some((PortName::E, 16))) => AdcMux::A, // e16:0-S0,1
            (0, 2,  Some((PortName::E, 18))) => AdcMux::A, // e18:0-S0,2
            (0, 3,  Some((PortName::E, 22))) => AdcMux::A, // e22:0-S0,3
            (0, 4,  Some((PortName::E, 21))) => AdcMux::A, // e21:0-S0,4a
            (0, 4,  Some((PortName::E, 29))) => AdcMux::B, // e29:0-S0,4b
            (0, 5,  Some((PortName::E, 17))) => AdcMux::A, // e17:0-S0,5a
            (0, 5,  Some((PortName::D, 1)))  => AdcMux::B, // d1:0-S0,5b
            (0, 6,  Some((PortName::E, 19))) => AdcMux::A, // e19:0-S0,6a
            (0, 6,  Some((PortName::D, 5)))  => AdcMux::B, // d5:0-S0,6b
            (0, 7,  Some((PortName::E, 23))) => AdcMux::A, // e23:0-S0,7a
            (0, 7,  Some((PortName::D, 6)))  => AdcMux::B, // d6:0-S0,7b
            (0, 8,  Some((PortName::B, 0)))  => AdcMux::A, // b0:0-S0,8
            (0, 9,  Some((PortName::B, 1)))  => AdcMux::A, // b1:0-S0,9
            (0, 11, Some((PortName::C, 2)))  => AdcMux::A, // c2:0-S0,11
            (0, 12, Some((PortName::B, 2)))  => AdcMux::A, // b2:0-S0,12
            (0, 13, Some((PortName::B, 3)))  => AdcMux::A, // b3:0-S0,13
            (0, 14, Some((PortName::C, 0)))  => AdcMux::A, // c0:0-S0,14
            (0, 15, Some((PortName::C, 1)))  => AdcMux::A, // c1:0-S0,15
            (0, 23, Some((PortName::E, 30))) => AdcMux::A, // e30:0-S0,23

            (_, 26, None) => AdcMux::A, // Single-ended Temp Sensor
            (_, 27, None) => AdcMux::A, // Single-ended Bandgap
            (_, 29, None) => AdcMux::A, // Single-ended V_REFSH
            (_, 30, None) => AdcMux::A, // Single-ended V_REFSL
            //(_, 31, None) => AdcMux::A, // Disabled

            (_, _, _) => return Err(()),
        };

        let reg = match id {
            0 => &mut *(ADC0_ADDR as *mut AdcRegs),
            _ => return Err(())
        };

        reg.cfg1.modify(|mut cfg1| {
            cfg1.set_bits(5..7, clkdiv as u32);
            cfg1.set_bits(2..4, resolution as u32);
            cfg1
        });

        reg.cfg2.modify(|mut cfg2| {
            cfg2.set_bit(4, mux.value());
            cfg2
        });

        reg.sc2.modify(|mut sc2| {
            sc2.set_bits(0..1, vref as u32);
            sc2
        });

        reg.sc1a.modify(|mut sc1a| {
            sc1a.set_bit(5, false);
            sc1a.set_bits(0..5, u32::from(ch));
            sc1a
        });

        Ok(Adc { reg, _pin: pin, _gate: gate })
    }

    pub fn calibrate(&mut self) -> Result<u32, ()> {
        unsafe {
            self.reg.sc3.modify(|mut sc3| {
                sc3.set_bit(7, true);
                sc3
            });
            while self.reg.sc3.read().get_bit(7) {}

            if self.reg.sc3.read().get_bit(6) {
                return Err(())
            }
        }

        let mut calib = unsafe {
            self.reg.clp0.read() +
            self.reg.clp1.read() +
            self.reg.clp2.read() +
            self.reg.clp3.read() +
            self.reg.clp4.read() +
            self.reg.clps.read()
        };
        calib >>= 1;
        calib |= 0x8000;
        unsafe { self.reg.pg.write(calib); }

        Ok(calib)
    }

    pub fn set_calib(&mut self, calib: u32) {
        unsafe { self.reg.pg.write(calib); }
    }

    pub fn start_conv(&mut self) {
        unsafe { self.reg.sc1a.modify(|sc1a| sc1a); }
    }

    pub fn is_conv_done(&mut self) -> bool {
        unsafe { self.reg.sc1a.read().get_bit(7) }
    }

    pub fn read(&mut self) -> u32 {
        unsafe { self.reg.ra.read() }
    }
}

impl<'a,'b> AdcDiff<'a,'b> {
    pub unsafe fn new(id: u8,
                      ch: u8,
                      resolution: Resolution,
                      clkdiv: Divisor,
                      vref: VoltageRef,
                      pos: Option<AdcDiffPPin<'a>>,
                      neg: Option<AdcDiffMPin<'b>>,
                      gate: ClockGate)
                      -> Result<AdcDiff<'a,'b>, ()> {
        let pos_info = pos.as_ref().map(|p| (p.port_name(), p.pin()));
        let neg_info = neg.as_ref().map(|p| (p.port_name(), p.pin()));
        match (id, ch, pos_info, neg_info) {
            // ADC0
            // e20:0-D0,0P
            // e21:0-D0,0M
            (0, 0, Some((PortName::E, 20)), Some((PortName::E, 21))) |
            // e16:0-D0,1P
            // e17:0-D0,1M
            (0, 1, Some((PortName::E, 16)), Some((PortName::E, 17))) |
            // e18:0-D0,1P
            // e19:0-D0,1M
            (0, 2, Some((PortName::E, 18)), Some((PortName::E, 19))) |
            // e22:0-D0,3P
            // e23:0-D0,3M
            (0, 3, Some((PortName::E, 22)), Some((PortName::E, 23))) |

            // Differential Temp Sensor
            (_, 26, None, None) |
            // Differential Bandgap
            (_, 27, None, None) |
            // Differential V_REFSH
            (_, 29, None, None) => {}
            //(_, 31, None) => AdcMux::A, // Disabled

            (_, _, _, _) => return Err(())
        }

        let reg = match id {
            0 => &mut *(ADC0_ADDR as *mut AdcRegs),
            _ => return Err(())
        };

        reg.cfg1.modify(|mut cfg1| {
            cfg1.set_bits(5..7, clkdiv as u32);
            cfg1.set_bits(2..4, resolution as u32);
            cfg1
        });

        reg.sc2.modify(|mut sc2| {
            sc2.set_bits(0..1, vref as u32);
            sc2
        });

        reg.sc1a.modify(|mut sc1a| {
            sc1a.set_bit(5, true);
            sc1a.set_bits(0..5, u32::from(ch));
            sc1a
        });

        Ok(AdcDiff { reg, _pos: pos, _neg: neg, _gate: gate })
    }

    pub fn calibrate(&mut self) -> Result<(u32, u32), ()> {
        unsafe {
            self.reg.sc3.modify(|mut sc3| {
                sc3.set_bit(7, true);
                sc3
            });
            while self.reg.sc3.read().get_bit(7) {}

            if self.reg.sc3.read().get_bit(6) {
                return Err(())
            }
        }

        let mut calib_p = unsafe {
            self.reg.clp0.read() +
            self.reg.clp1.read() +
            self.reg.clp2.read() +
            self.reg.clp3.read() +
            self.reg.clp4.read() +
            self.reg.clps.read()
        };
        calib_p >>= 1;
        calib_p |= 0x8000;
        unsafe { self.reg.pg.write(calib_p); }

        let mut calib_m = unsafe {
            self.reg.clm0.read() +
            self.reg.clm1.read() +
            self.reg.clm2.read() +
            self.reg.clm3.read() +
            self.reg.clm4.read() +
            self.reg.clms.read()
        };
        calib_m >>= 1;
        calib_m |= 0x8000;
        unsafe { self.reg.mg.write(calib_m); }

        Ok((calib_p, calib_m))
    }

    pub fn set_calib(&mut self, calib: u32) {
        unsafe { self.reg.pg.write(calib); }
    }

    pub fn start_conv(&mut self) {
        unsafe { self.reg.sc1a.modify(|sc1a| sc1a); }
    }

    pub fn is_conv_done(&mut self) -> bool {
        unsafe { self.reg.sc1a.read().get_bit(7) }
    }

    pub fn read(&mut self) -> i32 {
        unsafe { self.reg.ra.read() as i32 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adc0_test() {
        unsafe {
            let reg = & *(ADC0_ADDR as *const AdcRegs);
            assert_eq!(0x4003_B000 as *const RW<u32>, &reg.sc1a as *const RW<u32>, "sc1a");
            assert_eq!(0x4003_B004 as *const RW<u32>, &reg.sc1b as *const RW<u32>, "sc1b");
            assert_eq!(0x4003_B008 as *const RW<u32>, &reg.cfg1 as *const RW<u32>, "cfg1");
            assert_eq!(0x4003_B00C as *const RW<u32>, &reg.cfg2 as *const RW<u32>, "cfg2");
            assert_eq!(0x4003_B010 as *const RO<u32>, &reg.ra   as *const RO<u32>, "ra");
            assert_eq!(0x4003_B014 as *const RO<u32>, &reg.rb   as *const RO<u32>, "rb");
            assert_eq!(0x4003_B018 as *const RW<u32>, &reg.cv1  as *const RW<u32>, "cv1");
            assert_eq!(0x4003_B01C as *const RW<u32>, &reg.cv2  as *const RW<u32>, "cv2");
            assert_eq!(0x4003_B020 as *const RW<u32>, &reg.sc2  as *const RW<u32>, "sc2");
            assert_eq!(0x4003_B024 as *const RW<u32>, &reg.sc3  as *const RW<u32>, "sc3");
            assert_eq!(0x4003_B028 as *const RW<u32>, &reg.ofs  as *const RW<u32>, "ofs");
            assert_eq!(0x4003_B02C as *const RW<u32>, &reg.pg   as *const RW<u32>, "pg");
            assert_eq!(0x4003_B030 as *const RW<u32>, &reg.mg   as *const RW<u32>, "mg");
            assert_eq!(0x4003_B034 as *const RW<u32>, &reg.clpd as *const RW<u32>, "clpd");
            assert_eq!(0x4003_B038 as *const RW<u32>, &reg.clps as *const RW<u32>, "clps");
            assert_eq!(0x4003_B03C as *const RW<u32>, &reg.clp4 as *const RW<u32>, "clp4");
            assert_eq!(0x4003_B040 as *const RW<u32>, &reg.clp3 as *const RW<u32>, "clp3");
            assert_eq!(0x4003_B044 as *const RW<u32>, &reg.clp2 as *const RW<u32>, "clp2");
            assert_eq!(0x4003_B048 as *const RW<u32>, &reg.clp1 as *const RW<u32>, "clp1");
            assert_eq!(0x4003_B04C as *const RW<u32>, &reg.clp0 as *const RW<u32>, "clp0");
            assert_eq!(0x4003_B054 as *const RW<u32>, &reg.clmd as *const RW<u32>, "clmd");
            assert_eq!(0x4003_B058 as *const RW<u32>, &reg.clms as *const RW<u32>, "clms");
            assert_eq!(0x4003_B05C as *const RW<u32>, &reg.clm4 as *const RW<u32>, "clm4");
            assert_eq!(0x4003_B060 as *const RW<u32>, &reg.clm3 as *const RW<u32>, "clm3");
            assert_eq!(0x4003_B064 as *const RW<u32>, &reg.clm2 as *const RW<u32>, "clm2");
            assert_eq!(0x4003_B068 as *const RW<u32>, &reg.clm1 as *const RW<u32>, "clm1");
            assert_eq!(0x4003_B06C as *const RW<u32>, &reg.clm0 as *const RW<u32>, "clm0");
        }
    }
}
