use bit_field::BitField;
use cortex_m::interrupt;
use embedded_hal::timer;
use void::Void;
use volatile_register::RW;

use crate::port::{PortName, TpmPin};
use crate::sim::ClockGate;

// KL26 manual - pp 570-571
const TPM0_ADDR: usize = 0x4003_8000;
const TPM1_ADDR: usize = 0x4003_9000;
const TPM2_ADDR: usize = 0x4003_A000;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TpmNum {
    TPM0 = 0,
    TPM1 = 1,
    TPM2 = 2,
}

#[repr(C, packed)]
struct TpmRegs {
    sc: RW<u32>,
    cnt: RW<u32>,
    mod_: RW<u32>,
    channel: [ChannelRegs; 6],
    _pad0: [u8; 20],
    status: RW<u32>,
    _pad1: [u8; 48],
    conf: RW<u32>,
}

#[repr(C, packed)]
struct ChannelRegs {
    cnsc: RW<u32>,
    cnv: RW<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PwmSelect {
    Up = 0,
    UpDown = 1,
}

#[derive(Clone, Copy, Debug)]
pub enum ClockMode {
    Disabled = 0,
    EveryClock = 1,
    ExternalCLock = 2,
}

#[derive(Clone, Copy, Debug)]
pub enum Prescale {
    Div1 = 0b000,
    Div2 = 0b001,
    Div4 = 0b010,
    Div8 = 0b011,
    Div16 = 0b100,
    Div32 = 0b101,
    Div64 = 0b110,
    Div128 = 0b111,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ChannelNum {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
    Ch3 = 3,
    Ch4 = 4,
    Ch5 = 5,
}

#[derive(Clone, Copy, Debug)]
pub enum CaptureEdge {
    Rising,
    Falling,
    Both,
}

#[derive(Clone, Copy, Debug)]
pub enum OutputBehavior {
    Toggle,
    Clear,
    Set,
}

#[derive(Clone, Copy, Debug)]
pub enum PwmDirection {
    LowTrue,
    HighTrue,
}

#[derive(Clone, Copy, Debug)]
pub enum PulseDirection {
    Low,
    High,
}

// See Table 31-34 for specific definitions of these modes.
#[derive(Clone, Copy, Debug)]
pub enum ChannelMode {
    SoftwareCompare,
    InputCapture(CaptureEdge),
    OutputCompare(OutputBehavior),
    EdgePwm(PwmDirection),
    PulseOutputCompare(PulseDirection),
    CenterPwm(PwmDirection),
}

#[derive(Clone, Copy, Debug)]
pub enum ChannelError {
    PinMismatchTpm,
    PinMismatchChannel,
    CenterAlignMismatch,
    ChannelNotDisabled,
}

pub struct TpmPeriodic {
    reg: &'static mut TpmRegs,
    name: TpmNum,
    _gate: ClockGate,
}

// TODO: impl timer::Cancel
impl TpmPeriodic {
    pub unsafe fn new(
        name: TpmNum,
        cpwms: PwmSelect,
        cmod: ClockMode,
        clkdivider: Prescale,
        interrupt: bool,
        count: u16,
        gate: ClockGate,
    ) -> TpmPeriodic {
        let reg = &mut *match name {
            TpmNum::TPM0 => TPM0_ADDR as *mut TpmRegs,
            TpmNum::TPM1 => TPM1_ADDR as *mut TpmRegs,
            TpmNum::TPM2 => TPM2_ADDR as *mut TpmRegs,
        };

        reg.sc.write(0);
        reg.cnt.write(0);

        reg.mod_.write(count as u32);

        // set SC values
        // 0 - DMA disable
        // 1 - Clear TOF flag
        // 0 - up-counting
        let mut sc = 0b0100_0000;
        sc.set_bit(6, interrupt);
        sc.set_bit(5, cpwms == PwmSelect::UpDown);
        sc.set_bits(0..3, clkdivider as u32);
        reg.sc.write(sc);

        reg.conf.modify(|mut conf| {
            conf.set_bit(17, false);
            conf
        });

        // Enable the TPM.
        reg.sc.modify(|mut sc| {
            sc.set_bits(3..5, cmod as u32);
            sc
        });

        TpmPeriodic {
            reg,
            name,
            _gate: gate,
        }
    }

    pub fn channel<'a, 'b, T, const N: PortName, const P: usize>(
        &'a self,
        channel: ChannelNum,
        mode: ChannelMode,
        interrupt: bool,
        value: u16,
        pin: T,
    ) -> Result<Channel<'a, 'b, N, P>, ChannelError>
    where
        T: Into<Option<TpmPin<'b, N, P>>>,
    {
        let pin = pin.into();
        if let Some(pin) = pin.as_ref() {
            if pin.tpm() != self.name {
                return Err(ChannelError::PinMismatchTpm);
            }

            if pin.ch() != channel {
                return Err(ChannelError::PinMismatchChannel);
            }
        }

        unsafe {
            // According to Table 31-34 of the MKL26 Reference Manual v3.3
            // If Center-aligned PWM is selected but CPWMS is not set, error.
            // Same if any other mode is selected when CPWMS is set.
            // Software Compare doesn't care for some reason.
            match (mode, self.reg.sc.read().get_bit(5)) {
                (ChannelMode::CenterPwm(_), false) => {
                    return Err(ChannelError::CenterAlignMismatch)
                }
                (ChannelMode::SoftwareCompare, _) => {}
                (_, true) => return Err(ChannelError::CenterAlignMismatch),
                _ => {}
            }

            let channel_reg = &self.reg.channel[channel as usize];

            interrupt::free(|_| Channel::new(channel_reg, mode, interrupt, value, pin))
        }
    }

    pub fn set_clock_mode(&mut self, clock_mode: ClockMode) {
        unsafe {
            self.reg.sc.modify(|mut sc| {
                sc.set_bits(3..5, clock_mode as u32);
                sc
            });
        }
    }

    pub fn reset_count(&mut self) {
        unsafe {
            self.reg.cnt.write(0);
        }
    }

    pub fn set_period(&mut self, period: u16) {
        unsafe {
            self.reg.mod_.write(period as u32);
        }
    }

    pub fn is_overflowed(&self) -> bool {
        unsafe { self.reg.sc.read().get_bit(7) }
    }

    pub fn clear_overflow(&mut self) {
        unsafe {
            self.reg.sc.modify(|mut sc| {
                sc.set_bit(7, true);
                sc
            });
        }
    }
}

impl timer::CountDown for TpmPeriodic {
    type Time = u16;

    fn start<T>(&mut self, time: T)
    where
        T: Into<Self::Time>,
    {
        unsafe {
            let old_sc = self.reg.sc.read();

            // disable
            self.reg.sc.modify(|mut sc| {
                // clear the timer overflow flag
                sc.set_bit(7, true);
                sc.set_bits(3..5, ClockMode::Disabled as u32);
                sc
            });

            // clear the counter
            self.reg.cnt.write(0);
            // Set mod value
            self.reg.mod_.write(time.into() as u32);

            // enable
            self.reg.sc.write(old_sc);
        }
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        unsafe {
            if self.reg.sc.read().get_bit(7) {
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }
}

impl timer::Periodic for TpmPeriodic {}

pub struct TpmSingleShot {
    reg: &'static mut TpmRegs,
    name: TpmNum,
    _gate: ClockGate,
}

// TODO: impl timer::Cancel
impl TpmSingleShot {
    pub unsafe fn new(
        name: TpmNum,
        cpwms: PwmSelect,
        cmod: ClockMode,
        clkdivider: Prescale,
        interrupt: bool,
        count: u16,
        gate: ClockGate,
    ) -> TpmSingleShot {
        let reg = &mut *match name {
            TpmNum::TPM0 => TPM0_ADDR as *mut TpmRegs,
            TpmNum::TPM1 => TPM1_ADDR as *mut TpmRegs,
            TpmNum::TPM2 => TPM2_ADDR as *mut TpmRegs,
        };

        reg.sc.write(0);
        reg.cnt.write(0);

        reg.mod_.write(count as u32);

        // set SC values
        // 0 - DMA disable
        // 1 - Clear TOF flag
        // 0 - up-counting
        let mut sc = 0b0100_0000;
        sc.set_bit(6, interrupt);
        sc.set_bit(5, cpwms == PwmSelect::UpDown);
        sc.set_bits(0..3, clkdivider as u32);
        reg.sc.write(sc);

        reg.conf.modify(|mut conf| {
            conf.set_bit(17, true);
            conf
        });

        // Enable the TPM.
        reg.sc.modify(|mut sc| {
            sc.set_bits(3..5, cmod as u32);
            sc
        });

        TpmSingleShot {
            reg,
            name,
            _gate: gate,
        }
    }

    pub fn channel<'a, 'b, T, const N: PortName, const P: usize>(
        &'a self,
        channel: ChannelNum,
        mode: ChannelMode,
        interrupt: bool,
        value: u16,
        pin: T,
    ) -> Result<Channel<'a, 'b, N, P>, ChannelError>
    where
        T: Into<Option<TpmPin<'b, N, P>>>,
    {
        let pin = pin.into();
        if let Some(pin) = pin.as_ref() {
            if pin.tpm() != self.name {
                return Err(ChannelError::PinMismatchTpm);
            }

            if pin.ch() != channel {
                return Err(ChannelError::PinMismatchChannel);
            }
        }

        unsafe {
            // According to Table 31-34 of the MKL26 Reference Manual v3.3
            // If Center-aligned PWM is selected but CPWMS is not set, error.
            // Same if any other mode is selected when CPWMS is set.
            // Software Compare doesn't care for some reason.
            match (mode, self.reg.sc.read().get_bit(5)) {
                (ChannelMode::CenterPwm(_), false) => {
                    return Err(ChannelError::CenterAlignMismatch)
                }
                (ChannelMode::SoftwareCompare, _) => {}
                (_, true) => return Err(ChannelError::CenterAlignMismatch),
                _ => {}
            }

            let channel_reg = &self.reg.channel[channel as usize];

            interrupt::free(|_| Channel::new(channel_reg, mode, interrupt, value, pin))
        }
    }

    pub fn set_clock_mode(&mut self, clock_mode: ClockMode) {
        unsafe {
            self.reg.sc.modify(|mut sc| {
                sc.set_bits(3..5, clock_mode as u32);
                sc
            });
        }
    }

    pub fn reset_count(&mut self) {
        unsafe {
            self.reg.cnt.write(0);
        }
    }

    pub fn set_period(&mut self, period: u16) {
        unsafe {
            self.reg.mod_.write(period as u32);
        }
    }

    pub fn is_overflowed(&self) -> bool {
        unsafe { self.reg.sc.read().get_bit(7) }
    }

    pub fn clear_overflow(&mut self) {
        unsafe {
            self.reg.sc.modify(|mut sc| {
                sc.set_bit(7, true);
                sc
            });
        }
    }
}

impl timer::CountDown for TpmSingleShot {
    type Time = u16;

    fn start<T>(&mut self, time: T)
    where
        T: Into<Self::Time>,
    {
        let old_sc = unsafe { self.reg.sc.read() };

        self.set_clock_mode(ClockMode::Disabled);
        self.clear_overflow();

        self.reset_count();
        self.set_period(time.into());

        unsafe {
            self.reg.sc.write(old_sc);
        }
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.is_overflowed() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

// TODO: separate channel modes into different structs
// TODO: impl embedded_hal timer traits
pub struct Channel<'a, 'b, const N: PortName, const P: usize> {
    reg: &'a ChannelRegs,
    _pin: Option<TpmPin<'b, N, P>>,
}

impl<'a, 'b, const N: PortName, const P: usize> Channel<'a, 'b, N, P> {
    unsafe fn new(
        reg: &'a ChannelRegs,
        mode: ChannelMode,
        interrupt: bool,
        value: u16,
        pin: Option<TpmPin<'b, N, P>>,
    ) -> Result<Channel<'a, 'b, N, P>, ChannelError> {
        // Checking both ELSx and MSx.
        if reg.cnsc.read().get_bits(2..6) != 0b0000 {
            // Channel not currently disabled.
            return Err(ChannelError::ChannelNotDisabled);
        }

        reg.cnv.write(value as u32);

        let (mode, edge) = match mode {
            ChannelMode::SoftwareCompare => (0b01, 0b00),
            ChannelMode::InputCapture(edge) => {
                let edge = match edge {
                    CaptureEdge::Rising => 0b01,
                    CaptureEdge::Falling => 0b10,
                    CaptureEdge::Both => 0b11,
                };
                (0b00, edge)
            }
            ChannelMode::OutputCompare(edge) => {
                let edge = match edge {
                    OutputBehavior::Toggle => 0b01,
                    OutputBehavior::Clear => 0b10,
                    OutputBehavior::Set => 0b11,
                };
                (0b01, edge)
            }
            ChannelMode::EdgePwm(edge) => {
                let edge = match edge {
                    PwmDirection::HighTrue => 0b10,
                    PwmDirection::LowTrue => 0b01,
                };
                (0b10, edge)
            }
            ChannelMode::PulseOutputCompare(edge) => {
                let edge = match edge {
                    PulseDirection::Low => 0b10,
                    PulseDirection::High => 0b01,
                };
                (0b11, edge)
            }
            ChannelMode::CenterPwm(edge) => {
                let edge = match edge {
                    PwmDirection::HighTrue => 0b10,
                    PwmDirection::LowTrue => 0b01,
                };
                (0b10, edge)
            }
        };

        let mut cnsc = 0;
        cnsc.set_bit(7, true); // Clear CHF
        cnsc.set_bit(6, interrupt); // CHIE
        cnsc.set_bits(4..6, mode as u8); // set channel mode
        cnsc.set_bits(2..4, edge as u8); // edge and level selection

        //0b1110_1000 works
        reg.cnsc.write(cnsc as u32);

        Ok(Channel { reg, _pin: pin })
    }

    pub fn get_value(&self) -> u16 {
        unsafe {
            // Only the bottom 16 bits of the channel value register are used so truncation is fine.
            self.reg.cnv.read() as u16
        }
    }

    pub fn set_value(&mut self, channel_val: u16) {
        unsafe {
            self.reg.cnv.write(channel_val as u32);
        }
    }

    pub fn is_triggered(&mut self) -> bool {
        unsafe { self.reg.cnsc.read().get_bit(7) }
    }

    pub fn clear_trigger(&mut self) {
        unsafe {
            self.reg.cnsc.modify(|mut cnsc| {
                cnsc.set_bit(7, true);
                cnsc
            });
        }
    }
}

impl<'a, 'b, const N: PortName, const P: usize> Drop for Channel<'a, 'b, N, P> {
    fn drop(&mut self) {
        unsafe {
            self.reg.cnsc.write(0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn tpm0_test() {
        unsafe {
            let reg = & *(TPM0_ADDR as *const TpmRegs);
            assert_eq!(0x4003_8000 as *const RW<u32>, &reg.sc               as *const RW<u32>, "sc");
            assert_eq!(0x4003_8004 as *const RW<u32>, &reg.cnt              as *const RW<u32>, "cnt");
            assert_eq!(0x4003_8008 as *const RW<u32>, &reg.mod_             as *const RW<u32>, "mod_");
            assert_eq!(0x4003_800C as *const RW<u32>, &reg.channel[0].cnsc  as *const RW<u32>, "c0sc");
            assert_eq!(0x4003_8010 as *const RW<u32>, &reg.channel[0].cnv   as *const RW<u32>, "c0v");
            assert_eq!(0x4003_8014 as *const RW<u32>, &reg.channel[1].cnsc  as *const RW<u32>, "c1sc");
            assert_eq!(0x4003_8018 as *const RW<u32>, &reg.channel[1].cnv   as *const RW<u32>, "c1v");
            assert_eq!(0x4003_801C as *const RW<u32>, &reg.channel[2].cnsc  as *const RW<u32>, "c2sc");
            assert_eq!(0x4003_8020 as *const RW<u32>, &reg.channel[2].cnv   as *const RW<u32>, "c2v");
            assert_eq!(0x4003_8024 as *const RW<u32>, &reg.channel[3].cnsc  as *const RW<u32>, "c3sc");
            assert_eq!(0x4003_8028 as *const RW<u32>, &reg.channel[3].cnv   as *const RW<u32>, "c3v");
            assert_eq!(0x4003_802C as *const RW<u32>, &reg.channel[4].cnsc  as *const RW<u32>, "c4sc");
            assert_eq!(0x4003_8030 as *const RW<u32>, &reg.channel[4].cnv   as *const RW<u32>, "c4v");
            assert_eq!(0x4003_8034 as *const RW<u32>, &reg.channel[5].cnsc  as *const RW<u32>, "c5sc");
            assert_eq!(0x4003_8038 as *const RW<u32>, &reg.channel[5].cnv   as *const RW<u32>, "c5v");
            assert_eq!(0x4003_8050 as *const RW<u32>, &reg.status       as *const RW<u32>, "tpm_status");
            assert_eq!(0x4003_8084 as *const RW<u32>, &reg.conf         as *const RW<u32>, "tpm_conf");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn tpm1_test() {
        unsafe {
            let reg = & *(TPM1_ADDR as *const TpmRegs);
            assert_eq!(0x4003_9000 as *const RW<u32>, &reg.sc               as *const RW<u32>, "sc");
            assert_eq!(0x4003_9004 as *const RW<u32>, &reg.cnt              as *const RW<u32>, "cnt");
            assert_eq!(0x4003_9008 as *const RW<u32>, &reg.mod_             as *const RW<u32>, "mod_");
            assert_eq!(0x4003_900C as *const RW<u32>, &reg.channel[0].cnsc  as *const RW<u32>, "c0sc");
            assert_eq!(0x4003_9010 as *const RW<u32>, &reg.channel[0].cnv   as *const RW<u32>, "c0v");
            assert_eq!(0x4003_9014 as *const RW<u32>, &reg.channel[1].cnsc  as *const RW<u32>, "c1sc");
            assert_eq!(0x4003_9018 as *const RW<u32>, &reg.channel[1].cnv   as *const RW<u32>, "c1v");
            assert_eq!(0x4003_901C as *const RW<u32>, &reg.channel[2].cnsc  as *const RW<u32>, "c2sc");
            assert_eq!(0x4003_9020 as *const RW<u32>, &reg.channel[2].cnv   as *const RW<u32>, "c2v");
            assert_eq!(0x4003_9024 as *const RW<u32>, &reg.channel[3].cnsc  as *const RW<u32>, "c3sc");
            assert_eq!(0x4003_9028 as *const RW<u32>, &reg.channel[3].cnv   as *const RW<u32>, "c3v");
            assert_eq!(0x4003_902C as *const RW<u32>, &reg.channel[4].cnsc  as *const RW<u32>, "c4sc");
            assert_eq!(0x4003_9030 as *const RW<u32>, &reg.channel[4].cnv   as *const RW<u32>, "c4v");
            assert_eq!(0x4003_9034 as *const RW<u32>, &reg.channel[5].cnsc  as *const RW<u32>, "c5sc");
            assert_eq!(0x4003_9038 as *const RW<u32>, &reg.channel[5].cnv   as *const RW<u32>, "c5v");
            assert_eq!(0x4003_9050 as *const RW<u32>, &reg.status       as *const RW<u32>, "tpm_status");
            assert_eq!(0x4003_9084 as *const RW<u32>, &reg.conf         as *const RW<u32>, "tpm_conf");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn tpm2_test() {
        unsafe {
            let reg = & *(TPM2_ADDR as *const TpmRegs);
            assert_eq!(0x4003_A000 as *const RW<u32>, &reg.sc               as *const RW<u32>, "sc");
            assert_eq!(0x4003_A004 as *const RW<u32>, &reg.cnt              as *const RW<u32>, "cnt");
            assert_eq!(0x4003_A008 as *const RW<u32>, &reg.mod_             as *const RW<u32>, "mod_");
            assert_eq!(0x4003_A00C as *const RW<u32>, &reg.channel[0].cnsc  as *const RW<u32>, "c0sc");
            assert_eq!(0x4003_A010 as *const RW<u32>, &reg.channel[0].cnv   as *const RW<u32>, "c0v");
            assert_eq!(0x4003_A014 as *const RW<u32>, &reg.channel[1].cnsc  as *const RW<u32>, "c1sc");
            assert_eq!(0x4003_A018 as *const RW<u32>, &reg.channel[1].cnv   as *const RW<u32>, "c1v");
            assert_eq!(0x4003_A01C as *const RW<u32>, &reg.channel[2].cnsc  as *const RW<u32>, "c2sc");
            assert_eq!(0x4003_A020 as *const RW<u32>, &reg.channel[2].cnv   as *const RW<u32>, "c2v");
            assert_eq!(0x4003_A024 as *const RW<u32>, &reg.channel[3].cnsc  as *const RW<u32>, "c3sc");
            assert_eq!(0x4003_A028 as *const RW<u32>, &reg.channel[3].cnv   as *const RW<u32>, "c3v");
            assert_eq!(0x4003_A02C as *const RW<u32>, &reg.channel[4].cnsc  as *const RW<u32>, "c4sc");
            assert_eq!(0x4003_A030 as *const RW<u32>, &reg.channel[4].cnv   as *const RW<u32>, "c4v");
            assert_eq!(0x4003_A034 as *const RW<u32>, &reg.channel[5].cnsc  as *const RW<u32>, "c5sc");
            assert_eq!(0x4003_A038 as *const RW<u32>, &reg.channel[5].cnv   as *const RW<u32>, "c5v");
            assert_eq!(0x4003_A050 as *const RW<u32>, &reg.status       as *const RW<u32>, "tpm_status");
            assert_eq!(0x4003_A084 as *const RW<u32>, &reg.conf         as *const RW<u32>, "tpm_conf");
        }
    }
}
