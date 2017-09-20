use volatile::Volatile;
use bit_field::BitField;

use port::{AdcPin,PortName};
use sim::ClockGate;

#[repr(C,packed)]
struct AdcRegs {
    sc1a: Volatile<u32>,
    sc1b: Volatile<u32>,
    cfg1: Volatile<u32>,
    cfg2: Volatile<u32>,
    ra: Volatile<u32>,
    rb: Volatile<u32>,
    cv1: Volatile<u32>,
    cv2: Volatile<u32>,
    sc2: Volatile<u32>,
    sc3: Volatile<u32>,
    ofs: Volatile<u32>,
    pg: Volatile<u32>,
    mg: Volatile<u32>,
    clpd: Volatile<u32>,
    clps: Volatile<u32>,
    clp4: Volatile<u32>,
    clp3: Volatile<u32>,
    clp2: Volatile<u32>,
    clp1: Volatile<u32>,
    clp0: Volatile<u32>,
    pga: Volatile<u32>,
    clmd: Volatile<u32>,
    clms: Volatile<u32>,
    clm4: Volatile<u32>,
    clm3: Volatile<u32>,
    clm2: Volatile<u32>,
    clm1: Volatile<u32>,
    clm0: Volatile<u32>,
}

pub struct Adc<'a> {
    reg: &'static mut AdcRegs,
    _pin: Option<AdcPin<'a>>,
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
        let mux = match (id, ch) {
            // ADC0
            (0, 0) => match pin_info {
                None => AdcMux::A, // 9-??:0-D0,0P
                _ => return Err(())
            },
            (0, 2) => match pin_info {
                None => AdcMux::A, // 9-??:0-P0,P
                _ => return Err(())
            },
            (0, 3) => match pin_info {
                None => AdcMux::A, // 11-??:0-D0,3P
                _ => return Err(())
            },
            (0, 4) => match pin_info {
                Some((PortName::C, 2)) => AdcMux::B, // 45-c2:0-S0,4b
                _ => return Err(())
            },
            (0, 5) => match pin_info {
                Some((PortName::D, 1)) => AdcMux::B, // 58-d1:0-S0,5b
                _ => return Err(())
            },
            (0, 6) => match pin_info {
                Some((PortName::D, 5)) => AdcMux::B, // 62-d5:0-S0,6b
                _ => return Err(())
            },
            (0, 7) => match pin_info {
                Some((PortName::D, 6)) => AdcMux::B, // 63-d6:0-S0,7b
                _ => return Err(())
            },
            (0, 8) => match pin_info {
                Some((PortName::B, 0)) => AdcMux::A, // 35-b0:0-S0,8
                _ => return Err(())
            },
            (0, 9) => match pin_info {
                Some((PortName::B, 1)) => AdcMux::A, // 36-b1:0-S0,9
                _ => return Err(())
            },
            (0, 12) => match pin_info {
                Some((PortName::B, 2)) => AdcMux::A, // 37-b2:0-S0,12
                _ => return Err(())
            },
            (0, 13) => match pin_info {
                Some((PortName::B, 3)) => AdcMux::A, // 38-b3:0-S0,13
                _ => return Err(())
            },
            (0, 14) => match pin_info {
                Some((PortName::C, 0)) => AdcMux::A, // 43-c0:0-S0,14
                _ => return Err(())
            },
            (0, 15) => match pin_info {
                Some((PortName::C, 1)) => AdcMux::A, // 44-c1:0-S0,15
                _ => return Err(())
            },
            (0, 19) => match pin_info {
                None => AdcMux::A, // 10-??:0-D0,0M
                _ => return Err(())
            },
            (0, 21) => match pin_info {
                None => AdcMux::A, // 12-??:0-D0,3M
                _ => return Err(())
            },
            (0, 22) => match pin_info {
                None => AdcMux::A, // VREF Output
                _ => return Err(())
            },
            (0, 23) => match pin_info {
                None => AdcMux::A, // 18-??:?-S0,23 / 12-bit DAC0 Output
                _ => return Err(())
            },

            // ADC1
            (1, 0) => match pin_info {
                None => AdcMux::A, // 11-??:0-D1,0P
                _ => return Err(())
            },
            (1, 2) => match pin_info {
                None => AdcMux::A, // 11-??:0-P1,P
                _ => return Err(())
            },
            (1, 3) => match pin_info {
                None => AdcMux::A, // 9-??:0-D1,3P
                _ => return Err(())
            },
            (1, 4) => match pin_info {
                Some((PortName::E, 0)) => AdcMux::A, //  1-e0:0-S1,4a
                Some((PortName::C, 8)) => AdcMux::B, // 53-c8:0-S1,4b
                _ => return Err(())
            },
            (1, 5) => match pin_info {
                Some((PortName::E, 1)) => AdcMux::A, //  2-e1:0-S1,5a
                Some((PortName::C, 9)) => AdcMux::B, // 54-c9:0-S1,5b
                _ => return Err(())
            },
            (1, 6) => match pin_info {
                Some((PortName::C, 10)) => AdcMux::B, // 55-c10:0-S1,6b
                _ => return Err(())
            },
            (1, 7) => match pin_info {
                Some((PortName::C, 11)) => AdcMux::B, // 56-c11:0-S1,7b
                _ => return Err(())
            },
            (1, 8) => match pin_info {
                Some((PortName::B, 0)) => AdcMux::A, // 35-b0:0-S1,8
                _ => return Err(())
            },
            (1, 9) => match pin_info {
                Some((PortName::B, 1)) => AdcMux::A, // 36-b1:0-S1,9
                _ => return Err(())
            },
            (1, 18) => match pin_info {
                None => AdcMux::A, // 17-??:?-S1,18 / VREF Output
                _ => return Err(())
            },
            (1, 19) => match pin_info {
                None => AdcMux::A, // 12-??:0-D1,0M
                _ => return Err(())
            },

            (_, 26) => match pin_info {
                None => AdcMux::A, // Single-ended Temp Sensor
                _ => return Err(())
            },
            (_, 27) => match pin_info {
                None => AdcMux::A, // Single-ended Bandgap
                _ => return Err(())
            },
            (_, 29) => match pin_info {
                None => AdcMux::A, // Single-ended V_REFSH
                _ => return Err(())
            }
            (_, 30) => match pin_info {
                None => AdcMux::A, // Single-ended V_REFSL
                _ => return Err(())
            }
            /*
            (_, 31) => match pin_info {
                None => AdcMux::A, // Disabled
                _ => return Err(())
            }
            */

            (_, _) => unimplemented!()
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

        reg.cfg1.update(|cfg1| {
            cfg1.set_bits(5..7, clkdiv);
            cfg1.set_bits(2..4, mode);
        });

        reg.cfg2.update(|cfg2| {
            cfg2.set_bit(4, mux.value());
        });

        reg.sc1a.update(|sc1a| {
            sc1a.set_bit(5, false);
            sc1a.set_bits(0..5, ch as u32);
        });

        Ok(Adc {reg: reg, _pin: pin, _gate: gate})
    }

    pub fn calibrate(&mut self) -> Result<u32, ()> {
        self.reg.sc3.update(|sc3| {
            sc3.set_bit(7, true);
        });
        while self.reg.sc3.read().get_bit(7) {}

        if self.reg.sc3.read().get_bit(6) {
            return Err(())
        }

        let mut calib = self.reg.clp0.read() +
                        self.reg.clp1.read() +
                        self.reg.clp2.read() +
                        self.reg.clp3.read() +
                        self.reg.clp4.read() +
                        self.reg.clps.read();
        calib >>= 1;
        calib |= 0x8000;
        self.reg.pg.write(calib);

        Ok(calib)
    }

    pub fn set_calib(&mut self, calib: u32) {
        self.reg.pg.write(calib);
    }

    pub fn start_conv(&mut self) {
        self.reg.sc1a.update(|_| {})
    }

    pub fn is_conv_done(&mut self) -> bool {
        self.reg.sc1a.read().get_bit(7)
    }

    pub fn read(&mut self) -> u32 {
        self.reg.ra.read()
    }
}
