use core::cell::UnsafeCell;

use bit_field::BitField;
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use void::Void;
use volatile_register::{RO, RW, WO};

use crate::atomic::InterruptAtomic;
use crate::sim::ClockGate;
use crate::tpm::{ChannelNum, TpmNum};
use crate::uart::UartNum;

#[derive(Clone, Copy, Eq, PartialEq)]
pub enum PortName {
    A,
    B,
    C,
    D,
    E,
}

#[repr(C, packed)]
struct PortRegs {
    pcr: [RW<u32>; 32],
    gpclr: WO<u32>,
    gpchr: WO<u32>,
    reserved_0: [u8; 24],
    isfr: RW<u32>,
}

const PORT_A_ADDR: usize = 0x4004_9000;
const PORT_B_ADDR: usize = 0x4004_A000;
const PORT_C_ADDR: usize = 0x4004_B000;
const PORT_D_ADDR: usize = 0x4004_C000;
const PORT_E_ADDR: usize = 0x4004_D000;

pub struct Port<const N: PortName> {
    reg: UnsafeCell<&'static mut PortRegs>,
    locks: [InterruptAtomic<bool>; 32],
    _gate: ClockGate,
}

impl<const N: PortName> Port<N> {
    pub unsafe fn new(gate: ClockGate) -> Port<N> {
        let reg = &mut *match N {
            PortName::A => PORT_A_ADDR as *mut PortRegs,
            PortName::B => PORT_B_ADDR as *mut PortRegs,
            PortName::C => PORT_C_ADDR as *mut PortRegs,
            PortName::D => PORT_D_ADDR as *mut PortRegs,
            PortName::E => PORT_E_ADDR as *mut PortRegs,
        };

        Port {
            reg: UnsafeCell::new(reg),
            locks: Default::default(),
            _gate: gate,
        }
    }

    pub fn pin<const P: PinNum>(&self) -> Pin<N, P> {
        let was_init = self.locks[P as usize].swap(true);
        if was_init {
            panic!("Pin {} is already in use", P as usize);
        }
        Pin { port: self }
    }

    unsafe fn drop_pin(&self, p: PinNum) {
        self.locks[p as usize].store(false)
    }

    fn reg(&self) -> &'static mut PortRegs {
        // NOTE: This does no validation. It's on the calling
        // functions to ensure they're not accessing the same
        // registers from multiple codepaths. If they can't make those
        // guarantees, they should be marked as `unsafe` (See
        // `set_pin_mode` as an example).
        unsafe { *self.reg.get() }
    }
}

#[derive(Clone, Copy, Eq, PartialEq)]
pub enum PinNum {
    P0 = 0,
    P1 = 1,
    P2 = 2,
    P3 = 3,
    P4 = 4,
    P5 = 5,
    P6 = 6,
    P7 = 7,
    P8 = 8,
    P9 = 9,
    P10 = 10,
    P11 = 11,
    P12 = 12,
    P13 = 13,
    P14 = 14,
    P15 = 15,
    P16 = 16,
    P17 = 17,
    P18 = 18,
    P19 = 19,
    P20 = 20,
    P21 = 21,
    P22 = 22,
    P23 = 23,
    P24 = 24,
    P25 = 25,
    P26 = 26,
    P27 = 27,
    P28 = 28,
    P29 = 29,
    P30 = 30,
    P31 = 31,
}

pub struct Pin<'a, const N: PortName, const P: PinNum> {
    port: &'a Port<N>,
}

#[derive(Clone, Copy, PartialEq)]
pub enum DriveStrength {
    Low = 0,
    High = 1,
}

#[derive(Clone, Copy, PartialEq)]
pub enum SlewRate {
    Fast = 0,
    Slow = 1,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Pull {
    Disable = 0,
    Down = 2,
    Up = 3,
}

#[derive(Clone, Copy, PartialEq)]
pub enum InterruptConfig {
    Disabled = 0x0,
    DmaRising = 0x1,
    DmaFalling = 0x2,
    DmaEither = 0x3,
    LogicZero = 0x8,
    Rising = 0x9,
    Falling = 0xA,
    Either = 0xB,
    LogicOne = 0xC,
}

impl<'a, const N: PortName, const P: PinNum> Pin<'a, N, P> {
    fn set_mode(&mut self, mode: u32) {
        unsafe {
            self.port.reg().pcr[P as usize].modify(|mut pcr| {
                pcr.set_bits(8..11, mode);
                pcr
            });
        }
    }

    pub fn drive_strength(&mut self, strength: DriveStrength) {
        unsafe {
            self.port.reg().pcr[P as usize].modify(|mut pcr| {
                pcr.set_bit(6, strength == DriveStrength::High);
                pcr
            });
        }
    }

    pub fn open_drain(&mut self, enable: bool) {
        unsafe {
            self.port.reg().pcr[P as usize].modify(|mut pcr| {
                pcr.set_bit(5, enable);
                pcr
            });
        }
    }

    pub fn slew_rate(&mut self, rate: SlewRate) {
        unsafe {
            self.port.reg().pcr[P as usize].modify(|mut pcr| {
                pcr.set_bit(2, rate == SlewRate::Slow);
                pcr
            });
        }
    }

    pub fn pull(&mut self, pull: Pull) {
        unsafe {
            self.port.reg().pcr[P as usize].modify(|mut pcr| {
                pcr.set_bits(0..2, pull as u32);
                pcr
            });
        }
    }

    pub fn interrupt_config(&mut self, config: InterruptConfig) {
        unsafe {
            self.port.reg().pcr[P as usize].modify(|mut pcr| {
                pcr.set_bits(16..20, config as u32);
                pcr
            });
        }
    }

    pub fn is_interrupted(&self) -> bool {
        unsafe { self.port.reg().pcr[P as usize].read().get_bit(24) }
    }

    pub fn clear_interrupt(&mut self) {
        unsafe {
            self.port.reg().pcr[P as usize].modify(|mut pcr| {
                pcr.set_bit(24, true);
                pcr
            });
        }
    }

    pub fn to_gpio(mut self) -> Gpio<'a, N, P> {
        unsafe {
            self.set_mode(1);
            Gpio::new(self)
        }
    }
}

impl<'a, const N: PortName, const P: PinNum> Pin<'a, N, P> {
    pub fn to_adc(mut self) -> Result<AdcPin<'a, N, P>, ()> {
        match (N, P) {
            // e16:0-S0,1
            (PortName::E, PinNum::P16) |
            // e17:0-S0,5a
            (PortName::E, PinNum::P17) |
            // e18:0-S0,2
            (PortName::E, PinNum::P18) |
            // e19:0-S0,6a
            (PortName::E, PinNum::P19) |
            // e20:0-S0,0
            (PortName::E, PinNum::P20) |
            // e21:0-S0,4a
            (PortName::E, PinNum::P21) |
            // e22:0-S0,3
            (PortName::E, PinNum::P22) |
            // e23:0-S0,7a
            (PortName::E, PinNum::P23) |
            // e29:0-S0,4b
            (PortName::E, PinNum::P29) |
            // e30:0-S0,23
            (PortName::E, PinNum::P30) |
            // b0:0-S0,8
            (PortName::B, PinNum::P0) |
            // b1:0-S0,9
            (PortName::B, PinNum::P1) |
            // b2:0-S0,12
            (PortName::B, PinNum::P2) |
            // b3:0-S0,13
            (PortName::B, PinNum::P3) |
            // c0:0-S0,14
            (PortName::C, PinNum::P0) |
            // c1:0-S0,15
            (PortName::C, PinNum::P1) |
            // c2:0-S0,11
            (PortName::C, PinNum::P2) |
            // d1:0-S0,5b
            (PortName::D, PinNum::P1) |
            // d5:0-S0,6b
            (PortName::D, PinNum::P5) |
            // d6:0-S0,7b
            (PortName::D, PinNum::P6) => {
                self.set_mode(0);
                Ok(AdcPin { _pin: self })
            },
            _ => Err(())
        }
    }

    pub fn to_adc_diff_p(mut self) -> Result<AdcDiffPPin<'a, N, P>, ()> {
        match (N, P) {
            // e16:0-D0,1P
            (PortName::E, PinNum::P16) |
            // e18:0-D0,1P
            (PortName::E, PinNum::P18) |
            //  9-e20:0-D0,0P
            (PortName::E, PinNum::P20) |
            // 11-e22:0-D0,3P
            (PortName::E, PinNum::P22) => {
                self.set_mode(0);
                Ok(AdcDiffPPin { _pin: self })
            }
            _ => Err(())
        }
    }

    pub fn to_adc_diff_m(mut self) -> Result<AdcDiffMPin<'a, N, P>, ()> {
        match (N, P) {
            // e17:0-D0,1M
            (PortName::E, PinNum::P17) |
            // e19:0-D0,1M
            (PortName::E, PinNum::P19) |
            // 10-e21:0-D0,0M
            (PortName::E, PinNum::P21) |
            // 12-e23:0-D0,3M
            (PortName::E, PinNum::P23) => {
                self.set_mode(0);
                Ok(AdcDiffMPin { _pin: self })
            }
            _ => Err(())
        }
    }

    // TODO: maybe make the bool an enum ot make it clear at the callsite what is going on.
    pub fn to_i2c_scl(mut self, pullup_ext: bool) -> Result<I2cScl<'a, N, P>, ()> {
        let bus = match (N, P) {
            (PortName::E, PinNum::P1) => {
                self.set_mode(6);
                1
            }
            (PortName::E, PinNum::P24) => {
                self.set_mode(5);
                0
            }
            (PortName::A, PinNum::P3) => {
                self.set_mode(2);
                1
            }
            (PortName::B, PinNum::P0) => {
                self.set_mode(2);
                0
            }
            (PortName::B, PinNum::P2) => {
                self.set_mode(2);
                0
            }
            (PortName::C, PinNum::P1) => {
                self.set_mode(2);
                1
            }
            (PortName::C, PinNum::P8) => {
                self.set_mode(2);
                0
            }
            (PortName::C, PinNum::P10) => {
                self.set_mode(2);
                1
            }
            _ => return Err(()),
        };

        if pullup_ext {
            self.drive_strength(DriveStrength::High);
            self.open_drain(true);
            self.slew_rate(SlewRate::Slow);
            self.pull(Pull::Disable);
        } else {
            self.drive_strength(DriveStrength::Low);
            self.open_drain(false);
            self.slew_rate(SlewRate::Fast);
            self.pull(Pull::Up);
        }

        Ok(I2cScl {
            i2c: bus,
            _pin: self,
        })
    }

    // TODO: maybe make the bool an enum ot make it clear at the callsite what is going on.
    pub fn to_i2c_sda(mut self, pullup_ext: bool) -> Result<I2cSda<'a, N, P>, ()> {
        let bus = match (N, P) {
            (PortName::E, PinNum::P0) => {
                self.set_mode(6);
                1
            }
            (PortName::E, PinNum::P25) => {
                self.set_mode(5);
                0
            }
            (PortName::A, PinNum::P4) => {
                self.set_mode(2);
                1
            }
            (PortName::B, PinNum::P1) => {
                self.set_mode(2);
                0
            }
            (PortName::B, PinNum::P3) => {
                self.set_mode(2);
                0
            }
            (PortName::C, PinNum::P2) => {
                self.set_mode(2);
                1
            }
            (PortName::C, PinNum::P9) => {
                self.set_mode(2);
                0
            }
            (PortName::C, PinNum::P11) => {
                self.set_mode(2);
                1
            }
            _ => return Err(()),
        };

        if pullup_ext {
            self.open_drain(true);
            self.slew_rate(SlewRate::Slow);
            self.drive_strength(DriveStrength::High);
            self.pull(Pull::Disable);
        } else {
            self.open_drain(false);
            self.slew_rate(SlewRate::Fast);
            self.drive_strength(DriveStrength::Low);
            self.pull(Pull::Up);
        }

        Ok(I2cSda {
            i2c: bus,
            _pin: self,
        })
    }

    pub fn to_spi_mosi(mut self) -> Result<SpiMosi<'a, N, P>, ()> {
        match (N, P) {
            (PortName::E, PinNum::P1) => {
                self.set_mode(2);
                Ok(SpiMosi { spi: 1, _pin: self })
            }
            (PortName::E, PinNum::P3) => {
                self.set_mode(5);
                Ok(SpiMosi { spi: 1, _pin: self })
            }
            (PortName::E, PinNum::P18) => {
                self.set_mode(2);
                Ok(SpiMosi { spi: 0, _pin: self })
            }
            (PortName::E, PinNum::P19) => {
                self.set_mode(5);
                Ok(SpiMosi { spi: 0, _pin: self })
            }
            (PortName::A, PinNum::P16) => {
                self.set_mode(2);
                Ok(SpiMosi { spi: 0, _pin: self })
            }
            (PortName::A, PinNum::P17) => {
                self.set_mode(5);
                Ok(SpiMosi { spi: 0, _pin: self })
            }
            (PortName::B, PinNum::P16) => {
                self.set_mode(2);
                Ok(SpiMosi { spi: 1, _pin: self })
            }
            (PortName::B, PinNum::P17) => {
                self.set_mode(5);
                Ok(SpiMosi { spi: 1, _pin: self })
            }
            (PortName::C, PinNum::P6) => {
                self.set_mode(2);
                Ok(SpiMosi { spi: 0, _pin: self })
            }
            (PortName::C, PinNum::P7) => {
                self.set_mode(5);
                Ok(SpiMosi { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P2) => {
                self.set_mode(2);
                Ok(SpiMosi { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P3) => {
                self.set_mode(5);
                Ok(SpiMosi { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P6) => {
                self.set_mode(2);
                Ok(SpiMosi { spi: 1, _pin: self })
            }
            (PortName::D, PinNum::P7) => {
                self.set_mode(5);
                Ok(SpiMosi { spi: 1, _pin: self })
            }
            _ => Err(()),
        }
    }

    pub fn to_spi_miso(mut self) -> Result<SpiMiso<'a, N, P>, ()> {
        match (N, P) {
            (PortName::E, PinNum::P0) => {
                self.set_mode(2);
                Ok(SpiMiso { spi: 1, _pin: self })
            }
            (PortName::E, PinNum::P1) => {
                self.set_mode(5);
                Ok(SpiMiso { spi: 1, _pin: self })
            }
            (PortName::E, PinNum::P3) => {
                self.set_mode(2);
                Ok(SpiMiso { spi: 1, _pin: self })
            }
            (PortName::E, PinNum::P18) => {
                self.set_mode(5);
                Ok(SpiMiso { spi: 0, _pin: self })
            }
            (PortName::E, PinNum::P19) => {
                self.set_mode(2);
                Ok(SpiMiso { spi: 0, _pin: self })
            }
            (PortName::A, PinNum::P16) => {
                self.set_mode(5);
                Ok(SpiMiso { spi: 0, _pin: self })
            }
            (PortName::A, PinNum::P17) => {
                self.set_mode(2);
                Ok(SpiMiso { spi: 0, _pin: self })
            }
            (PortName::B, PinNum::P16) => {
                self.set_mode(5);
                Ok(SpiMiso { spi: 1, _pin: self })
            }
            (PortName::B, PinNum::P17) => {
                self.set_mode(2);
                Ok(SpiMiso { spi: 1, _pin: self })
            }
            (PortName::C, PinNum::P6) => {
                self.set_mode(5);
                Ok(SpiMiso { spi: 0, _pin: self })
            }
            (PortName::C, PinNum::P7) => {
                self.set_mode(2);
                Ok(SpiMiso { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P2) => {
                self.set_mode(5);
                Ok(SpiMiso { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P3) => {
                self.set_mode(2);
                Ok(SpiMiso { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P6) => {
                self.set_mode(5);
                Ok(SpiMiso { spi: 1, _pin: self })
            }
            (PortName::D, PinNum::P7) => {
                self.set_mode(2);
                Ok(SpiMiso { spi: 1, _pin: self })
            }
            _ => Err(()),
        }
    }

    pub fn to_spi_sck(mut self) -> Result<SpiSck<'a, N, P>, ()> {
        match (N, P) {
            (PortName::E, PinNum::P2) => {
                self.set_mode(2);
                Ok(SpiSck { spi: 1, _pin: self })
            }
            (PortName::E, PinNum::P17) => {
                self.set_mode(2);
                Ok(SpiSck { spi: 0, _pin: self })
            }
            (PortName::A, PinNum::P15) => {
                self.set_mode(2);
                Ok(SpiSck { spi: 0, _pin: self })
            }
            (PortName::B, PinNum::P9) => {
                self.set_mode(2);
                Ok(SpiSck { spi: 1, _pin: self })
            }
            (PortName::B, PinNum::P11) => {
                self.set_mode(2);
                Ok(SpiSck { spi: 1, _pin: self })
            }
            (PortName::C, PinNum::P5) => {
                self.set_mode(2);
                Ok(SpiSck { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P1) => {
                self.set_mode(2);
                Ok(SpiSck { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P5) => {
                self.set_mode(2);
                Ok(SpiSck { spi: 1, _pin: self })
            }
            _ => Err(()),
        }
    }

    pub fn to_spi_cs(mut self) -> Result<SpiCs<'a, N, P>, ()> {
        match (N, P) {
            (PortName::E, PinNum::P4) => {
                self.set_mode(2);
                Ok(SpiCs { spi: 1, _pin: self })
            }
            (PortName::E, PinNum::P16) => {
                self.set_mode(2);
                Ok(SpiCs { spi: 0, _pin: self })
            }
            (PortName::A, PinNum::P14) => {
                self.set_mode(2);
                Ok(SpiCs { spi: 0, _pin: self })
            }
            (PortName::B, PinNum::P8) => {
                self.set_mode(2);
                Ok(SpiCs { spi: 1, _pin: self })
            }
            (PortName::B, PinNum::P10) => {
                self.set_mode(2);
                Ok(SpiCs { spi: 1, _pin: self })
            }
            (PortName::C, PinNum::P4) => {
                self.set_mode(2);
                Ok(SpiCs { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P0) => {
                self.set_mode(2);
                Ok(SpiCs { spi: 0, _pin: self })
            }
            (PortName::D, PinNum::P4) => {
                self.set_mode(2);
                Ok(SpiCs { spi: 1, _pin: self })
            }
            _ => Err(()),
        }
    }

    pub fn to_tpm(mut self) -> Result<TpmPin<'a, N, P>, ()> {
        match (N, P) {
            // e20:3-1,0
            (PortName::E, PinNum::P20) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM1,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // e21:3-1,1
            (PortName::E, PinNum::P21) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM1,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // e22:3-2,0
            (PortName::E, PinNum::P22) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM2,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // e23:3-2,1
            (PortName::E, PinNum::P23) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM2,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // e29:3-0,2
            (PortName::E, PinNum::P29) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch2,
                    _pin: self,
                })
            }
            // e30:3-0,3
            (PortName::E, PinNum::P30) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch3,
                    _pin: self,
                })
            }
            // e31:3-0,4
            (PortName::E, PinNum::P31) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch4,
                    _pin: self,
                })
            }
            // e24:3-0,0
            (PortName::E, PinNum::P24) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // e25:3-0,1
            (PortName::E, PinNum::P25) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // e26:3-0,5
            (PortName::E, PinNum::P26) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch5,
                    _pin: self,
                })
            }
            // a0:3-0,5
            (PortName::A, PinNum::P0) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch5,
                    _pin: self,
                })
            }
            // a1:3-2,0
            (PortName::A, PinNum::P1) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM2,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // a2:3-2,1
            (PortName::A, PinNum::P2) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM2,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // a3:3-0,0
            (PortName::A, PinNum::P3) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // a4:3-0,1
            (PortName::A, PinNum::P4) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // a5:3-0,2
            (PortName::A, PinNum::P5) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch2,
                    _pin: self,
                })
            }
            // a6:3-0,3
            (PortName::A, PinNum::P6) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch3,
                    _pin: self,
                })
            }
            // a7:3-0,4
            (PortName::A, PinNum::P7) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch4,
                    _pin: self,
                })
            }
            // a12:3-1,0
            (PortName::A, PinNum::P12) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM1,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // a13:3-1,1
            (PortName::A, PinNum::P13) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM1,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // b0:3-1,0
            (PortName::B, PinNum::P0) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM1,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // b1:3-1,1
            (PortName::B, PinNum::P1) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM1,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // b2:3-2,0
            (PortName::B, PinNum::P2) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM2,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // b3:3-2,1
            (PortName::B, PinNum::P3) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM2,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // b18:3-2,0
            (PortName::B, PinNum::P18) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM2,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // b19:3-2,1
            (PortName::B, PinNum::P19) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM2,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }

            // c1:4-0,0
            (PortName::C, PinNum::P1) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // c2:4-0,1
            (PortName::C, PinNum::P2) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // c3:4-0,2
            (PortName::C, PinNum::P3) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch2,
                    _pin: self,
                })
            }
            // c4:4-0,3
            (PortName::C, PinNum::P4) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch3,
                    _pin: self,
                })
            }

            // c8:3-0,4
            (PortName::C, PinNum::P8) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch4,
                    _pin: self,
                })
            }
            // c9:3-0,5
            (PortName::C, PinNum::P9) => {
                self.set_mode(3);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch5,
                    _pin: self,
                })
            }

            // d0:4-0,0
            (PortName::D, PinNum::P0) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch0,
                    _pin: self,
                })
            }
            // d1:4-0,1
            (PortName::D, PinNum::P1) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch1,
                    _pin: self,
                })
            }
            // d2:4-0,2
            (PortName::D, PinNum::P2) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch2,
                    _pin: self,
                })
            }
            // d3:4-0,3
            (PortName::D, PinNum::P3) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch3,
                    _pin: self,
                })
            }
            // d4:4-0,4
            (PortName::D, PinNum::P4) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch4,
                    _pin: self,
                })
            }
            // d5:4-0,5
            (PortName::D, PinNum::P5) => {
                self.set_mode(4);
                Ok(TpmPin {
                    tpm: TpmNum::TPM0,
                    ch: ChannelNum::Ch5,
                    _pin: self,
                })
            }

            _ => Err(()),
        }
    }

    pub fn to_uart_rx(mut self) -> Result<UartRx<'a, N, P>, ()> {
        match (N, P) {
            (PortName::E, PinNum::P1) => {
                self.set_mode(3);
                Ok(UartRx {
                    uart: UartNum::UART1,
                    _pin: self,
                })
            }
            (PortName::E, PinNum::P17) => {
                self.set_mode(3);
                Ok(UartRx {
                    uart: UartNum::UART2,
                    _pin: self,
                })
            }
            (PortName::E, PinNum::P21) => {
                self.set_mode(4);
                Ok(UartRx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            (PortName::E, PinNum::P23) => {
                self.set_mode(4);
                Ok(UartRx {
                    uart: UartNum::UART2,
                    _pin: self,
                })
            }
            (PortName::A, PinNum::P1) => {
                self.set_mode(2);
                Ok(UartRx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            (PortName::A, PinNum::P15) => {
                self.set_mode(3);
                Ok(UartRx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            (PortName::A, PinNum::P18) => {
                self.set_mode(3);
                Ok(UartRx {
                    uart: UartNum::UART1,
                    _pin: self,
                })
            }
            (PortName::B, PinNum::P16) => {
                self.set_mode(3);
                Ok(UartRx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            (PortName::C, PinNum::P3) => {
                self.set_mode(3);
                Ok(UartRx {
                    uart: UartNum::UART1,
                    _pin: self,
                })
            }
            (PortName::D, PinNum::P2) => {
                self.set_mode(3);
                Ok(UartRx {
                    uart: UartNum::UART2,
                    _pin: self,
                })
            }
            (PortName::D, PinNum::P4) => {
                self.set_mode(3);
                Ok(UartRx {
                    uart: UartNum::UART2,
                    _pin: self,
                })
            }
            (PortName::D, PinNum::P6) => {
                self.set_mode(3);
                Ok(UartRx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            _ => Err(()),
        }
    }

    pub fn to_uart_tx(mut self) -> Result<UartTx<'a, N, P>, ()> {
        match (N, P) {
            (PortName::E, PinNum::P0) => {
                self.set_mode(3);
                Ok(UartTx {
                    uart: UartNum::UART1,
                    _pin: self,
                })
            }
            (PortName::E, PinNum::P16) => {
                self.set_mode(3);
                Ok(UartTx {
                    uart: UartNum::UART2,
                    _pin: self,
                })
            }
            (PortName::E, PinNum::P20) => {
                self.set_mode(4);
                Ok(UartTx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            (PortName::E, PinNum::P22) => {
                self.set_mode(4);
                Ok(UartTx {
                    uart: UartNum::UART2,
                    _pin: self,
                })
            }
            (PortName::A, PinNum::P2) => {
                self.set_mode(2);
                Ok(UartTx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            (PortName::A, PinNum::P14) => {
                self.set_mode(3);
                Ok(UartTx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            (PortName::A, PinNum::P19) => {
                self.set_mode(3);
                Ok(UartTx {
                    uart: UartNum::UART1,
                    _pin: self,
                })
            }
            (PortName::B, PinNum::P17) => {
                self.set_mode(3);
                Ok(UartTx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            (PortName::C, PinNum::P4) => {
                self.set_mode(3);
                Ok(UartTx {
                    uart: UartNum::UART1,
                    _pin: self,
                })
            }
            (PortName::D, PinNum::P3) => {
                self.set_mode(3);
                Ok(UartTx {
                    uart: UartNum::UART2,
                    _pin: self,
                })
            }
            (PortName::D, PinNum::P5) => {
                self.set_mode(3);
                Ok(UartTx {
                    uart: UartNum::UART2,
                    _pin: self,
                })
            }
            (PortName::D, PinNum::P7) => {
                self.set_mode(3);
                Ok(UartTx {
                    uart: UartNum::UART0,
                    _pin: self,
                })
            }
            _ => Err(()),
        }
    }
}

impl<'a, const N: PortName, const P: PinNum> Drop for Pin<'a, N, P> {
    fn drop(&mut self) {
        unsafe {
            self.port.drop_pin(P);
        }
    }
}

#[allow(dead_code)]
const GPIO_A_ADDR: usize = 0x400F_F000;
#[allow(dead_code)]
const GPIO_B_ADDR: usize = 0x400F_F040;
#[allow(dead_code)]
const GPIO_C_ADDR: usize = 0x400F_F080;
#[allow(dead_code)]
const GPIO_D_ADDR: usize = 0x400F_F0C0;
#[allow(dead_code)]
const GPIO_E_ADDR: usize = 0x400F_F100;
#[allow(dead_code)]
const FGPIO_A_ADDR: usize = 0xF800_0000;
#[allow(dead_code)]
const FGPIO_B_ADDR: usize = 0xF800_0040;
#[allow(dead_code)]
const FGPIO_C_ADDR: usize = 0xF800_0080;
#[allow(dead_code)]
const FGPIO_D_ADDR: usize = 0xF800_00C0;
#[allow(dead_code)]
const FGPIO_E_ADDR: usize = 0xF800_0100;

#[repr(C, packed)]
struct GpioRegs {
    pdor: RW<u32>,
    psor: WO<u32>,
    pcor: WO<u32>,
    ptor: WO<u32>,
    pdir: RO<u32>,
    pddr: RW<u32>,
}

// TODO: maybe split into input and output
pub struct Gpio<'a, const N: PortName, const P: PinNum> {
    gpio: *mut GpioRegs,
    pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: PinNum> Gpio<'a, N, P> {
    // TODO: BME?
    pub unsafe fn new(pin: Pin<'a, N, P>) -> Gpio<'a, N, P> {
        let gpio = match N {
            // GPIO
            #[cfg(not(feature = "fgpio"))]
            PortName::A => GPIO_A_ADDR as *mut GpioRegs,
            #[cfg(not(feature = "fgpio"))]
            PortName::B => GPIO_B_ADDR as *mut GpioRegs,
            #[cfg(not(feature = "fgpio"))]
            PortName::C => GPIO_C_ADDR as *mut GpioRegs,
            #[cfg(not(feature = "fgpio"))]
            PortName::D => GPIO_D_ADDR as *mut GpioRegs,
            #[cfg(not(feature = "fgpio"))]
            PortName::E => GPIO_E_ADDR as *mut GpioRegs,
            // FGPIO
            #[cfg(feature = "fgpio")]
            PortName::A => FGPIO_A_ADDR as *mut GpioRegs,
            #[cfg(feature = "fgpio")]
            PortName::B => FGPIO_B_ADDR as *mut GpioRegs,
            #[cfg(feature = "fgpio")]
            PortName::C => FGPIO_C_ADDR as *mut GpioRegs,
            #[cfg(feature = "fgpio")]
            PortName::D => FGPIO_D_ADDR as *mut GpioRegs,
            #[cfg(feature = "fgpio")]
            PortName::E => FGPIO_E_ADDR as *mut GpioRegs,
        };

        Gpio { gpio, pin }
    }

    pub fn input(&mut self) {
        unsafe {
            (*self.gpio).pddr.modify(|mut pddr| {
                pddr &= !(1 << P as u8);
                pddr
            });
        }
    }

    pub fn read(&self) -> bool {
        unsafe { (*self.gpio).pdir.read().get_bit(P as u8) }
    }

    pub fn is_interrupted(&self) -> bool {
        self.pin.is_interrupted()
    }

    pub fn clear_interrupt(&mut self) {
        self.pin.clear_interrupt()
    }

    pub fn output(&mut self) {
        unsafe {
            (*self.gpio).pddr.modify(|mut pddr| {
                pddr |= 1 << P as usize;
                pddr
            });
        }
    }

    pub fn high(&mut self) {
        unsafe {
            (*self.gpio).psor.write(1 << P as u8);
        }
    }

    pub fn low(&mut self) {
        unsafe {
            (*self.gpio).pcor.write(1 << P as u8);
        }
    }

    pub fn toggle(&mut self) {
        unsafe {
            (*self.gpio).ptor.write(1 << P as u8);
        }
    }
}

impl<'a, const N: PortName, const P: PinNum> InputPin for Gpio<'a, N, P> {
    type Error = Void;

    fn is_high(&self) -> Result<bool, Void> {
        Ok(self.read())
    }

    fn is_low(&self) -> Result<bool, Void> {
        Ok(!self.read())
    }
}

impl<'a, const N: PortName, const P: PinNum> OutputPin for Gpio<'a, N, P> {
    type Error = Void;

    fn set_high(&mut self) -> Result<(), Void> {
        self.high();

        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Void> {
        self.low();

        Ok(())
    }
}

impl<'a, const N: PortName, const P: PinNum> ToggleableOutputPin for Gpio<'a, N, P> {
    type Error = Void;

    fn toggle(&mut self) -> Result<(), Void> {
        self.toggle();

        Ok(())
    }
}

pub struct AdcPin<'a, const N: PortName, const P: PinNum> {
    _pin: Pin<'a, N, P>,
}

pub struct AdcDiffPPin<'a, const N: PortName, const P: PinNum> {
    _pin: Pin<'a, N, P>,
}

pub struct AdcDiffMPin<'a, const N: PortName, const P: PinNum> {
    _pin: Pin<'a, N, P>,
}

pub struct I2cScl<'a, const N: PortName, const P: PinNum> {
    i2c: u8,
    _pin: Pin<'a, N, P>,
}

pub struct I2cSda<'a, const N: PortName, const P: PinNum> {
    i2c: u8,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: PinNum> I2cScl<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.i2c
    }
}

impl<'a, const N: PortName, const P: PinNum> I2cSda<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.i2c
    }
}

pub struct SpiMosi<'a, const N: PortName, const P: PinNum> {
    spi: u8,
    _pin: Pin<'a, N, P>,
}

pub struct SpiMiso<'a, const N: PortName, const P: PinNum> {
    spi: u8,
    _pin: Pin<'a, N, P>,
}

pub struct SpiSck<'a, const N: PortName, const P: PinNum> {
    spi: u8,
    _pin: Pin<'a, N, P>,
}

pub struct SpiCs<'a, const N: PortName, const P: PinNum> {
    spi: u8,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: PinNum> SpiMosi<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.spi
    }
}

impl<'a, const N: PortName, const P: PinNum> SpiMiso<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.spi
    }
}

impl<'a, const N: PortName, const P: PinNum> SpiSck<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.spi
    }
}

impl<'a, const N: PortName, const P: PinNum> SpiCs<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.spi
    }
}

pub struct TpmPin<'a, const N: PortName, const P: PinNum> {
    tpm: TpmNum,
    ch: ChannelNum,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: PinNum> TpmPin<'a, N, P> {
    pub fn tpm(&self) -> TpmNum {
        self.tpm
    }

    pub fn ch(&self) -> ChannelNum {
        self.ch
    }
}

pub struct UartRx<'a, const N: PortName, const P: PinNum> {
    uart: UartNum,
    _pin: Pin<'a, N, P>,
}

pub struct UartTx<'a, const N: PortName, const P: PinNum> {
    uart: UartNum,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: PinNum> UartRx<'a, N, P> {
    pub fn bus(&self) -> UartNum {
        self.uart
    }
}

impl<'a, const N: PortName, const P: PinNum> UartTx<'a, N, P> {
    pub fn bus(&self) -> UartNum {
        self.uart
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn port_a_regs() {
        unsafe {
            let reg = &*(PORT_A_ADDR as *const PortRegs);
            assert_eq!(0x4004_9000 as *const RW<u32>, &reg.pcr      as *const RW<u32>, "pcr");
            assert_eq!(0x4004_9080 as *const WO<u32>, &reg.gpclr    as *const WO<u32>, "gpclr");
            assert_eq!(0x4004_9084 as *const WO<u32>, &reg.gpchr    as *const WO<u32>, "gpchr");
            assert_eq!(0x4004_90A0 as *const RW<u32>, &reg.isfr     as *const RW<u32>, "isfr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn port_b_regs() {
        unsafe {
            let reg = &*(PORT_B_ADDR as *const PortRegs);
            assert_eq!(0x4004_A000 as *const RW<u32>, &reg.pcr      as *const RW<u32>, "pcr");
            assert_eq!(0x4004_A080 as *const WO<u32>, &reg.gpclr    as *const WO<u32>, "gpclr");
            assert_eq!(0x4004_A084 as *const WO<u32>, &reg.gpchr    as *const WO<u32>, "gpchr");
            assert_eq!(0x4004_A0A0 as *const RW<u32>, &reg.isfr     as *const RW<u32>, "isfr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn port_c_regs() {
        unsafe {
            let reg = &*(PORT_C_ADDR as *const PortRegs);
            assert_eq!(0x4004_B000 as *const RW<u32>, &reg.pcr      as *const RW<u32>, "pcr");
            assert_eq!(0x4004_B080 as *const WO<u32>, &reg.gpclr    as *const WO<u32>, "gpclr");
            assert_eq!(0x4004_B084 as *const WO<u32>, &reg.gpchr    as *const WO<u32>, "gpchr");
            assert_eq!(0x4004_B0A0 as *const RW<u32>, &reg.isfr     as *const RW<u32>, "isfr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn port_d_regs() {
        unsafe {
            let reg = &*(PORT_D_ADDR as *const PortRegs);
            assert_eq!(0x4004_C000 as *const RW<u32>, &reg.pcr      as *const RW<u32>, "pcr");
            assert_eq!(0x4004_C080 as *const WO<u32>, &reg.gpclr    as *const WO<u32>, "gpclr");
            assert_eq!(0x4004_C084 as *const WO<u32>, &reg.gpchr    as *const WO<u32>, "gpchr");
            assert_eq!(0x4004_C0A0 as *const RW<u32>, &reg.isfr     as *const RW<u32>, "isfr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn port_e_regs() {
        unsafe {
            let reg = &*(PORT_E_ADDR as *const PortRegs);
            assert_eq!(0x4004_D000 as *const RW<u32>, &reg.pcr      as *const RW<u32>, "pcr");
            assert_eq!(0x4004_D080 as *const WO<u32>, &reg.gpclr    as *const WO<u32>, "gpclr");
            assert_eq!(0x4004_D084 as *const WO<u32>, &reg.gpchr    as *const WO<u32>, "gpchr");
            assert_eq!(0x4004_D0A0 as *const RW<u32>, &reg.isfr     as *const RW<u32>, "isfr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn gpio_a_regs() {
        unsafe {
            let reg = &*(GPIO_A_ADDR as *const GpioRegs);
            assert_eq!(0x400F_F000 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0x400F_F004 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0x400F_F008 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0x400F_F00C as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0x400F_F010 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0x400F_F014 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn gpio_b_regs() {
        unsafe {
            let reg = &*(GPIO_B_ADDR as *const GpioRegs);
            assert_eq!(0x400F_F040 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0x400F_F044 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0x400F_F048 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0x400F_F04C as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0x400F_F050 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0x400F_F054 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn gpio_c_regs() {
        unsafe {
            let reg = &*(GPIO_C_ADDR as *const GpioRegs);
            assert_eq!(0x400F_F080 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0x400F_F084 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0x400F_F088 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0x400F_F08C as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0x400F_F090 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0x400F_F094 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn gpio_d_regs() {
        unsafe {
            let reg = &*(GPIO_D_ADDR as *const GpioRegs);
            assert_eq!(0x400F_F0C0 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0x400F_F0C4 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0x400F_F0C8 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0x400F_F0CC as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0x400F_F0D0 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0x400F_F0D4 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn gpio_e_regs() {
        unsafe {
            let reg = &*(GPIO_E_ADDR as *const GpioRegs);
            assert_eq!(0x400F_F100 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0x400F_F104 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0x400F_F108 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0x400F_F10C as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0x400F_F110 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0x400F_F114 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn fgpio_a_regs() {
        unsafe {
            let reg = &*(FGPIO_A_ADDR as *const GpioRegs);
            assert_eq!(0xF800_0000 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0xF800_0004 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0xF800_0008 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0xF800_000C as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0xF800_0010 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0xF800_0014 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn fgpio_b_regs() {
        unsafe {
            let reg = &*(FGPIO_B_ADDR as *const GpioRegs);
            assert_eq!(0xF800_0040 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0xF800_0044 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0xF800_0048 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0xF800_004C as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0xF800_0050 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0xF800_0054 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn fgpio_c_regs() {
        unsafe {
            let reg = &*(FGPIO_C_ADDR as *const GpioRegs);
            assert_eq!(0xF800_0080 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0xF800_0084 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0xF800_0088 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0xF800_008C as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0xF800_0090 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0xF800_0094 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn fgpio_d_regs() {
        unsafe {
            let reg = &*(FGPIO_D_ADDR as *const GpioRegs);
            assert_eq!(0xF800_00C0 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0xF800_00C4 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0xF800_00C8 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0xF800_00CC as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0xF800_00D0 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0xF800_00D4 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn fgpio_e_regs() {
        unsafe {
            let reg = &*(FGPIO_E_ADDR as *const GpioRegs);
            assert_eq!(0xF800_0100 as *const RW<u32>, &reg.pdor as *const RW<u32>, "pdor");
            assert_eq!(0xF800_0104 as *const WO<u32>, &reg.psor as *const WO<u32>, "psor");
            assert_eq!(0xF800_0108 as *const WO<u32>, &reg.pcor as *const WO<u32>, "pcor");
            assert_eq!(0xF800_010C as *const WO<u32>, &reg.ptor as *const WO<u32>, "ptor");
            assert_eq!(0xF800_0110 as *const RO<u32>, &reg.pdir as *const RO<u32>, "pdir");
            assert_eq!(0xF800_0114 as *const RW<u32>, &reg.pddr as *const RW<u32>, "pddr");
        }
    }
}
