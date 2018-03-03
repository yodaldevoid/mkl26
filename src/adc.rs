use volatile_register::{RO,RW};
use bit_field::BitField;

use port::{AdcPin,AdcDiffPPin,AdcDiffMPin,PortName};
use sim::ClockGate;

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
    pga:    RW<u32>,
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

// 1.2 V VREF connected to V_ALTH/V_ALTL

impl<'a> Adc<'a> {
    pub unsafe fn new(id: u8,
                      ch: u8,
                      mode: u8,
                      clkdiv: u8,
                      pin: Option<AdcPin<'a>>,
                      gate: ClockGate)
                      -> Result<Adc<'a>, ()> {
        let pin_info = pin.as_ref().map(|p| (p.port_name(), p.pin()));
        let mux = match (id, ch, pin_info) {
            // ADC0
            (0, 0,  Some((PortName::E, 20))) => AdcMux::A, //  9-e20:0-S0,0
            (0, 3,  Some((PortName::E, 22))) => AdcMux::A, // 11-e22:0-S0,3
            (0, 4,  Some((PortName::E, 21))) => AdcMux::A, // 10-e21:0-S0,4a
            (0, 4,  Some((PortName::E, 29))) => AdcMux::B, // 17-e29:0-S0,4b
            (0, 5,  Some((PortName::D, 1)))  => AdcMux::B, // 58-d1:0-S0,5b
            (0, 6,  Some((PortName::D, 5)))  => AdcMux::B, // 62-d5:0-S0,6b
            (0, 7,  Some((PortName::E, 23))) => AdcMux::A, // 12-e23:0-S0,7a
            (0, 7,  Some((PortName::D, 6)))  => AdcMux::B, // 58-d6:0-S0,7b
            (0, 8,  Some((PortName::B, 0)))  => AdcMux::A, // 35-b0:0-S0,8
            (0, 9,  Some((PortName::B, 1)))  => AdcMux::A, // 36-b1:0-S0,9
            (0, 11, Some((PortName::C, 2)))  => AdcMux::A, // 45-c2:0-S0,11
            (0, 12, Some((PortName::B, 2)))  => AdcMux::A, // 37-b2:0-S0,12
            (0, 13, Some((PortName::B, 3)))  => AdcMux::A, // 38-b3:0-S0,13
            (0, 14, Some((PortName::C, 0)))  => AdcMux::A, // 43-c0:0-S0,14
            (0, 15, Some((PortName::C, 1)))  => AdcMux::A, // 44-c1:0-S0,15
            (0, 23, Some((PortName::E, 30))) => AdcMux::A, // 18-e30:0-S0,23

            (_, 26, None) => AdcMux::A, // Single-ended Temp Sensor
            (_, 27, None) => AdcMux::A, // Single-ended Bandgap
            (_, 29, None) => AdcMux::A, // Single-ended V_REFSH
            (_, 30, None) => AdcMux::A, // Single-ended V_REFSL
            //(_, 31, None) => AdcMux::A, // Disabled

            (_, _, _) => return Err(()),
        };

        let mode = match mode {
            8 => 0,
            12 => 1,
            10 => 2,
            16 => 3,
            _ => return Err(()),
        };

        let clkdiv = match clkdiv {
            1 => 0,
            2 => 1,
            4 => 2,
            8 => 3,
            _ => return Err(()),
        };

        let reg = match id {
            0 => &mut *(0x4003B000 as *mut AdcRegs),
            1 => &mut *(0x400BB000 as *mut AdcRegs),
            _ => return Err(())
        };

        reg.cfg1.modify(|mut cfg1| {
            cfg1.set_bits(5..7, clkdiv);
            cfg1.set_bits(2..4, mode);
            cfg1
        });

        reg.cfg2.modify(|mut cfg2| {
            cfg2.set_bit(4, mux.value());
            cfg2
        });

        reg.sc1a.modify(|mut sc1a| {
            sc1a.set_bit(5, false);
            sc1a.set_bits(0..5, ch as u32);
            sc1a
        });

        Ok(Adc {reg: reg, _pin: pin, _gate: gate})
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
                      mode: u8,
                      clkdiv: u8,
                      pos: Option<AdcDiffPPin<'a>>,
                      neg: Option<AdcDiffMPin<'b>>,
                      gate: ClockGate)
                      -> Result<AdcDiff<'a,'b>, ()> {
        let pos_info = pos.as_ref().map(|p| (p.port_name(), p.pin()));
        let neg_info = neg.as_ref().map(|p| (p.port_name(), p.pin()));
        match (id, ch, pos_info, neg_info) {
            // ADC0
            //  9-e20:0-D0,0P
            // 10-e21:0-D0,0M
            (0, 0, Some((PortName::E, 20)), Some((PortName::E, 21))) |
            // 11-e22:0-D0,3P
            // 12-e23:0-D0,3M
            (0, 3, Some((PortName::E, 22)), Some((PortName::E, 23))) |

            // Differential Temp Sensor
            (_, 26, None, None) |
            // Differential Bandgap
            (_, 27, None, None) |
            // Differential V_REFSH
            (_, 29, None, None) => {}

            (_, _, _, _) => return Err(())
        }

        let mode = match mode {
            9 => 0,
            13 => 1,
            11 => 2,
            16 => 3,
            _ => return Err(()),
        };

        let clkdiv = match clkdiv {
            1 => 0,
            2 => 1,
            4 => 2,
            8 => 3,
            _ => return Err(()),
        };

        let reg = match id {
            0 => &mut *(0x4003B000 as *mut AdcRegs),
            1 => &mut *(0x400BB000 as *mut AdcRegs),
            _ => return Err(())
        };

        reg.cfg1.modify(|mut cfg1| {
            cfg1.set_bits(5..7, clkdiv);
            cfg1.set_bits(2..4, mode);
            cfg1
        });

        reg.sc1a.modify(|mut sc1a| {
            sc1a.set_bit(5, true);
            sc1a.set_bits(0..5, ch as u32);
            sc1a
        });

        Ok(AdcDiff {reg: reg, _pos: pos, _neg: neg, _gate: gate})
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
