use bit_field::BitField;
use port::TpmPin;
use sim::ClockGate;
use volatile_register::RW;

// KL26 manual - pp 570-571
const TPM0_ADDR: usize = 0x4003_8000;
const TPM1_ADDR: usize = 0x4003_9000;
const TPM2_ADDR: usize = 0x4003_A000;

#[derive(Clone, Copy)]
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
    c0sc: RW<u32>,
    c0v: RW<u32>,
    c1sc: RW<u32>,
    c1v: RW<u32>,
    c2sc: RW<u32>,
    c2v: RW<u32>,
    c3sc: RW<u32>,
    c3v: RW<u32>,
    c4sc: RW<u32>,
    c4v: RW<u32>,
    c5sc: RW<u32>,
    c5v: RW<u32>,
    _pad0: [u8; 20],
    tpm_status: RW<u32>,
    _pad1: [u8; 48],
    tpm_conf: RW<u32>,
}

#[derive(PartialEq)]
pub enum PwmSelect {
    Up = 0,
    UpDown = 1,
}

pub enum ClockMode {
    Disabled = 0,
    EveryClock = 1,
    ExternalCLock = 2,
}

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

#[derive(Clone, Copy, Debug)]
pub enum ChannelSelect {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
    Ch3 = 3,
    Ch4 = 4,
    Ch5 = 5,
}

// See Table 31-34 for specific definitions of these modes.
pub enum ChannelMode {
    SoftwareCompare,
    InputCapture,
    OutputCompare,
    EdgePwm,
    PulseOutputCompare,
    CenterPwm,
}

#[derive(Debug)]
pub enum ChannelError {
    PinMismatchTpm,
    PinMismatchChannel,
    ChannelNotDisabled,
}

pub struct Tpm {
    reg:   &'static mut TpmRegs,
    name:   TpmNum,
    _gate: ClockGate,
}

impl Tpm {
    pub unsafe fn new(
        name: TpmNum,
        cpwms: PwmSelect,
        cmod: ClockMode,
        clkdivider: Prescale,
        count: u16,
        gate: ClockGate,
    ) -> Result<Tpm, ()> {
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
        // 1 - TOIE
        // 0 - up-counting
        let mut sc = 0b0110_0000;
        sc.set_bit(5, cpwms == PwmSelect::UpDown);
        sc.set_bits(0..3, clkdivider as u32);
        reg.sc.write(sc);

        // Enable the TPM.
        reg.sc.modify(|mut sc| {
            sc.set_bits(3..5, cmod as u32);
            sc
        });

        Ok(Tpm {
            reg,
            name,
            _gate: gate,
        })
    }

    pub fn channel<'a, P: Into<Option<TpmPin<'a>>>>(
        &mut self,
        channel: ChannelSelect,
        mode: ChannelMode,
        edge: u8,
        value: u16,
        pin: P,
    ) -> Result<Channel<'a>, ChannelError> {
        let pin = pin.into();
        if let Some(pin) = pin.as_ref() {
            if pin.tpm() != self.name as u8 {
                return Err(ChannelError::PinMismatchTpm);
            }

            if pin.ch() != channel as u8 {
                return Err(ChannelError::PinMismatchChannel);
            }
        }

        unsafe {
            let channel_reg = &*match channel {
                ChannelSelect::Ch0 => &self.reg.c0sc as *const RW<u32> as *const ChanelRegs,
                ChannelSelect::Ch1 => &self.reg.c1sc as *const RW<u32> as *const ChanelRegs,
                ChannelSelect::Ch2 => &self.reg.c2sc as *const RW<u32> as *const ChanelRegs,
                ChannelSelect::Ch3 => &self.reg.c3sc as *const RW<u32> as *const ChanelRegs,
                ChannelSelect::Ch4 => &self.reg.c4sc as *const RW<u32> as *const ChanelRegs,
                ChannelSelect::Ch5 => &self.reg.c5sc as *const RW<u32> as *const ChanelRegs,
            };

            // Checking both ELSx and MSx.
            if channel_reg.cxsc.read().get_bits(2..6) != 0b0000 {
                // Channel not currently disabled.
                return Err(ChannelError::ChannelNotDisabled);
            }

            // TODO: check center aligned bit if asking for center aligned mode.

            Ok(Channel::new(channel_reg, mode, edge, value, pin))
        }
    }

    pub fn set_period(&mut self, period: u16) {
        unsafe {
            self.reg.mod_.write(period as u32);
        }
    }
}

#[repr(C, packed)]
struct ChanelRegs {
    cxsc: RW<u32>,
    cxv:  RW<u32>,
}

pub struct Channel<'a> {
    reg: &'static ChanelRegs,
    _pin: Option<TpmPin<'a>>,
}

//TODO: More robust channel options using enums.
impl<'a> Channel<'a> {
    unsafe fn new(
        reg: &'static ChanelRegs,
        mode: ChannelMode,
        edge: u8,
        value: u16,
        pin: Option<TpmPin<'a>>
    ) -> Channel<'a> {
        reg.cxv.write(value as u32);

        let mode = match mode {
            ChannelMode::SoftwareCompare => 0b01,
            ChannelMode::InputCapture => 0b00,
            ChannelMode::OutputCompare => 0b01,
            ChannelMode::EdgePwm => 0b10,
            ChannelMode::PulseOutputCompare => 0b11,
            ChannelMode::CenterPwm => 0b10,
        };

        let mut cnsc = 0;
        cnsc.set_bit(7, true); // Clear CHF
        cnsc.set_bit(6, true); // CHIE = true
        cnsc.set_bits(4..6, mode as u8); // set channel mode
        cnsc.set_bits(2..4, edge as u8); // edge and level selection

        //0b1110_1000 works
        reg.cxsc.write(cnsc as u32);

        Channel { reg, _pin: pin }
    }

    pub fn get_value(&self) -> u16 {
        unsafe {
            // Only the bottom 16 bits of the channel value register are used to truncation is fine.
            self.reg.cxv.read() as u16
        }
    }

    pub fn set_value(&mut self, channel_val: u16) {
        unsafe {
            self.reg.cxv.write(channel_val as u32);
        }
    }
}

impl<'a> Drop for Channel<'a> {
    fn drop(&mut self) {
        unsafe {
            self.reg.cxsc.write(0);
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
            let reg = & *(TPM0_ADDR as *const TPMRegs);
            assert_eq!(0x4003_8000 as *const RW<u32>, &reg.sc           as *const RW<u32>, "sc");
            assert_eq!(0x4003_8004 as *const RW<u32>, &reg.cnt          as *const RW<u32>, "cnt");
            assert_eq!(0x4003_8008 as *const RW<u32>, &reg.mod_         as *const RW<u32>, "mod_");
            assert_eq!(0x4003_800C as *const RW<u32>, &reg.c0sc         as *const RW<u32>, "c0sc");
            assert_eq!(0x4003_8010 as *const RW<u32>, &reg.c0v          as *const RW<u32>, "c0v");
            assert_eq!(0x4003_8014 as *const RW<u32>, &reg.c1sc         as *const RW<u32>, "c1sc");
            assert_eq!(0x4003_8018 as *const RW<u32>, &reg.c1v          as *const RW<u32>, "c1v");
            assert_eq!(0x4003_801C as *const RW<u32>, &reg.c2sc         as *const RW<u32>, "c2sc");
            assert_eq!(0x4003_8020 as *const RW<u32>, &reg.c2v          as *const RW<u32>, "c2v");
            assert_eq!(0x4003_8024 as *const RW<u32>, &reg.c3sc         as *const RW<u32>, "c3sc");
            assert_eq!(0x4003_8028 as *const RW<u32>, &reg.c3v          as *const RW<u32>, "c3v");
            assert_eq!(0x4003_802C as *const RW<u32>, &reg.c4sc         as *const RW<u32>, "c4sc");
            assert_eq!(0x4003_8030 as *const RW<u32>, &reg.c4v          as *const RW<u32>, "c4v");
            assert_eq!(0x4003_8034 as *const RW<u32>, &reg.c5sc         as *const RW<u32>, "c5sc");
            assert_eq!(0x4003_8038 as *const RW<u32>, &reg.c5v          as *const RW<u32>, "c5v");
            assert_eq!(0x4003_8050 as *const RW<u32>, &reg.tpm_status   as *const RW<u32>, "tpm_status");
            assert_eq!(0x4003_8084 as *const RW<u32>, &reg.tpm_conf     as *const RW<u32>, "tpm_conf");
        }
    }
}
