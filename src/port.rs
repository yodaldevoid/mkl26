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

    pub fn pin<const P: usize>(&self) -> Pin<N, P> {
        let was_init = self.locks[P].swap(true);
        if was_init {
            panic!("Pin {} is already in use", P);
        }
        Pin { port: self }
    }

    unsafe fn drop_pin(&self, p: usize) {
        self.locks[p].store(false)
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

pub struct Pin<'a, const N: PortName, const P: usize> {
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

impl<'a, const N: PortName, const P: usize> Pin<'a, N, P> {
    fn set_mode(&mut self, mode: u32) {
        unsafe {
            self.port.reg().pcr[P].modify(|mut pcr| {
                pcr.set_bits(8..11, mode);
                pcr
            });
        }
    }

    pub fn drive_strength(&mut self, strength: DriveStrength) {
        unsafe {
            self.port.reg().pcr[P].modify(|mut pcr| {
                pcr.set_bit(6, strength == DriveStrength::High);
                pcr
            });
        }
    }

    pub fn open_drain(&mut self, enable: bool) {
        unsafe {
            self.port.reg().pcr[P].modify(|mut pcr| {
                pcr.set_bit(5, enable);
                pcr
            });
        }
    }

    pub fn slew_rate(&mut self, rate: SlewRate) {
        unsafe {
            self.port.reg().pcr[P].modify(|mut pcr| {
                pcr.set_bit(2, rate == SlewRate::Slow);
                pcr
            });
        }
    }

    pub fn pull(&mut self, pull: Pull) {
        unsafe {
            self.port.reg().pcr[P].modify(|mut pcr| {
                pcr.set_bits(0..2, pull as u32);
                pcr
            });
        }
    }

    pub fn interrupt_config(&mut self, config: InterruptConfig) {
        unsafe {
            self.port.reg().pcr[P].modify(|mut pcr| {
                pcr.set_bits(16..20, config as u32);
                pcr
            });
        }
    }

    pub fn is_interrupted(&self) -> bool {
        unsafe { self.port.reg().pcr[P].read().get_bit(24) }
    }

    pub fn clear_interrupt(&mut self) {
        unsafe {
            self.port.reg().pcr[P].modify(|mut pcr| {
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

impl<'a, const N: PortName, const P: usize> Drop for Pin<'a, N, P> {
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
pub struct Gpio<'a, const N: PortName, const P: usize> {
    gpio: *mut GpioRegs,
    pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> Gpio<'a, N, P> {
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

impl<'a, const N: PortName, const P: usize> InputPin for Gpio<'a, N, P> {
    type Error = Void;

    fn is_high(&self) -> Result<bool, Void> {
        Ok(self.read())
    }

    fn is_low(&self) -> Result<bool, Void> {
        Ok(!self.read())
    }
}

impl<'a, const N: PortName, const P: usize> OutputPin for Gpio<'a, N, P> {
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

impl<'a, const N: PortName, const P: usize> ToggleableOutputPin for Gpio<'a, N, P> {
    type Error = Void;

    fn toggle(&mut self) -> Result<(), Void> {
        self.toggle();

        Ok(())
    }
}

pub struct AdcPin<'a, const N: PortName, const P: usize> {
    _pin: Pin<'a, N, P>,
}

pub trait ToAdcPin<'a, const N: PortName, const P: usize> {
    fn to_adc(self) -> AdcPin<'a, N, P>;
}

macro_rules! impl_to_adc {
    ($pn:expr, $pp:expr) => {
        impl<'a> ToAdcPin<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_adc(mut self) -> AdcPin<'a, $pn, $pp> {
                self.set_mode(0);
                AdcPin { _pin: self }
            }
        }
    };
}

// e16:0-S0,1
impl_to_adc!(PortName::E, 16);
// e17:0-S0,5a
impl_to_adc!(PortName::E, 17);
// e18:0-S0,2
impl_to_adc!(PortName::E, 18);
// e19:0-S0,6a
impl_to_adc!(PortName::E, 19);
// e20:0-S0,0
impl_to_adc!(PortName::E, 20);
// e21:0-S0,4a
impl_to_adc!(PortName::E, 21);
// e22:0-S0,3
impl_to_adc!(PortName::E, 22);
// e23:0-S0,7a
impl_to_adc!(PortName::E, 23);
// e29:0-S0,4b
impl_to_adc!(PortName::E, 29);
// e30:0-S0,23
impl_to_adc!(PortName::E, 30);
// b0:0-S0,8
impl_to_adc!(PortName::B, 0);
// b1:0-S0,9
impl_to_adc!(PortName::B, 1);
// b2:0-S0,12
impl_to_adc!(PortName::B, 2);
// b3:0-S0,13
impl_to_adc!(PortName::B, 3);
// c0:0-S0,14
impl_to_adc!(PortName::C, 0);
// c1:0-S0,15
impl_to_adc!(PortName::C, 1);
// c2:0-S0,11
impl_to_adc!(PortName::C, 2);
// d1:0-S0,5b
impl_to_adc!(PortName::D, 1);
// d5:0-S0,6b
impl_to_adc!(PortName::D, 5);
// d6:0-S0,7b
impl_to_adc!(PortName::D, 6);

pub struct AdcDiffPPin<'a, const N: PortName, const P: usize> {
    _pin: Pin<'a, N, P>,
}

pub trait ToAdcDiffPPin<'a, const N: PortName, const P: usize> {
    fn to_adc_diff_p(self) -> AdcDiffPPin<'a, N, P>;
}

macro_rules! impl_to_adc_diff_p {
    ($pn:expr, $pp:expr) => {
        impl<'a> ToAdcDiffPPin<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_adc_diff_p(mut self) -> AdcDiffPPin<'a, $pn, $pp> {
                self.set_mode(0);
                AdcDiffPPin { _pin: self }
            }
        }
    };
}

// e16:0-D0,1P
impl_to_adc_diff_p!(PortName::E, 16);
// e18:0-D0,1P
impl_to_adc_diff_p!(PortName::E, 18);
//  9-e20:0-D0,0P
impl_to_adc_diff_p!(PortName::E, 20);
// 11-e22:0-D0,3P
impl_to_adc_diff_p!(PortName::E, 22);

pub struct AdcDiffMPin<'a, const N: PortName, const P: usize> {
    _pin: Pin<'a, N, P>,
}

pub trait ToAdcDiffMPin<'a, const N: PortName, const P: usize> {
    fn to_adc_diff_m(self) -> AdcDiffMPin<'a, N, P>;
}

macro_rules! impl_to_adc_diff_m {
    ($pn:expr, $pp:expr) => {
        impl<'a> ToAdcDiffMPin<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_adc_diff_m(mut self) -> AdcDiffMPin<'a, $pn, $pp> {
                self.set_mode(0);
                AdcDiffMPin { _pin: self }
            }
        }
    };
}

// e17:0-D0,1M
impl_to_adc_diff_m!(PortName::E, 17);
// e19:0-D0,1M
impl_to_adc_diff_m!(PortName::E, 19);
// 10-e21:0-D0,0M
impl_to_adc_diff_m!(PortName::E, 21);
// 12-e23:0-D0,3M
impl_to_adc_diff_m!(PortName::E, 23);

pub struct I2cScl<'a, const N: PortName, const P: usize> {
    i2c: u8,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> I2cScl<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.i2c
    }
}

pub trait ToI2cScl<'a, const N: PortName, const P: usize> {
    // TODO: maybe make the bool an enum ot make it clear at the callsite what is going on.
    fn to_i2c_scl(self, pullup_ext: bool) -> I2cScl<'a, N, P>;
}

macro_rules! impl_to_i2c_scl {
    ($pn:expr, $pp:expr, $mode:expr, $bus:expr) => {
        impl<'a> ToI2cScl<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_i2c_scl(mut self, pullup_ext: bool) -> I2cScl<'a, $pn, $pp> {
                self.set_mode($mode);

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

                I2cScl {
                    i2c: $bus,
                    _pin: self,
                }
            }
        }
    };
}

impl_to_i2c_scl!(PortName::E, 1, 6, 1);
impl_to_i2c_scl!(PortName::E, 24, 5, 0);
impl_to_i2c_scl!(PortName::A, 3, 2, 1);
impl_to_i2c_scl!(PortName::B, 0, 2, 0);
impl_to_i2c_scl!(PortName::B, 2, 2, 0);
impl_to_i2c_scl!(PortName::C, 1, 2, 1);
impl_to_i2c_scl!(PortName::C, 8, 2, 0);
impl_to_i2c_scl!(PortName::C, 10, 2, 1);

pub struct I2cSda<'a, const N: PortName, const P: usize> {
    i2c: u8,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> I2cSda<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.i2c
    }
}

pub trait ToI2cSda<'a, const N: PortName, const P: usize> {
    // TODO: maybe make the bool an enum ot make it clear at the callsite what is going on.
    fn to_i2c_sda(self, pullup_ext: bool) -> I2cSda<'a, N, P>;
}

macro_rules! impl_to_i2c_sda {
    ($pn:expr, $pp:expr, $mode:expr, $bus:expr) => {
        impl<'a> ToI2cSda<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_i2c_sda(mut self, pullup_ext: bool) -> I2cSda<'a, $pn, $pp> {
                self.set_mode($mode);

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

                I2cSda {
                    i2c: $bus,
                    _pin: self,
                }
            }
        }
    };
}

impl_to_i2c_sda!(PortName::E, 0, 6, 1);
impl_to_i2c_sda!(PortName::E, 25, 5, 0);
impl_to_i2c_sda!(PortName::A, 4, 2, 1);
impl_to_i2c_sda!(PortName::B, 1, 2, 0);
impl_to_i2c_sda!(PortName::B, 3, 2, 0);
impl_to_i2c_sda!(PortName::C, 2, 2, 1);
impl_to_i2c_sda!(PortName::C, 9, 2, 0);
impl_to_i2c_sda!(PortName::C, 11, 2, 1);

pub struct SpiMosi<'a, const N: PortName, const P: usize> {
    spi: u8,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> SpiMosi<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.spi
    }
}

pub trait ToSpiMosi<'a, const N: PortName, const P: usize> {
    fn to_spi_mosi(self) -> SpiMosi<'a, N, P>;
}

macro_rules! impl_to_spi_mosi {
    ($pn:expr, $pp:expr, $mode:expr, $bus:expr) => {
        impl<'a> ToSpiMosi<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_spi_mosi(mut self) -> SpiMosi<'a, $pn, $pp> {
                self.set_mode($mode);

                SpiMosi {
                    spi: $bus,
                    _pin: self,
                }
            }
        }
    };
}

impl_to_spi_mosi!(PortName::E, 1, 2, 1);
impl_to_spi_mosi!(PortName::E, 3, 5, 1);
impl_to_spi_mosi!(PortName::E, 18, 2, 0);
impl_to_spi_mosi!(PortName::E, 19, 5, 0);
impl_to_spi_mosi!(PortName::A, 16, 2, 0);
impl_to_spi_mosi!(PortName::A, 17, 5, 0);
impl_to_spi_mosi!(PortName::B, 16, 2, 1);
impl_to_spi_mosi!(PortName::B, 17, 5, 1);
impl_to_spi_mosi!(PortName::C, 6, 2, 0);
impl_to_spi_mosi!(PortName::C, 7, 5, 0);
impl_to_spi_mosi!(PortName::D, 2, 2, 0);
impl_to_spi_mosi!(PortName::D, 3, 5, 0);
impl_to_spi_mosi!(PortName::D, 6, 2, 1);
impl_to_spi_mosi!(PortName::D, 7, 5, 1);

pub struct SpiMiso<'a, const N: PortName, const P: usize> {
    spi: u8,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> SpiMiso<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.spi
    }
}

pub trait ToSpiMiso<'a, const N: PortName, const P: usize> {
    fn to_spi_miso(self) -> SpiMiso<'a, N, P>;
}

macro_rules! impl_to_spi_miso {
    ($pn:expr, $pp:expr, $mode:expr, $bus:expr) => {
        impl<'a> ToSpiMiso<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_spi_miso(mut self) -> SpiMiso<'a, $pn, $pp> {
                self.set_mode($mode);

                SpiMiso {
                    spi: $bus,
                    _pin: self,
                }
            }
        }
    };
}

impl_to_spi_miso!(PortName::E, 0, 2, 1);
impl_to_spi_miso!(PortName::E, 1, 5, 1);
impl_to_spi_miso!(PortName::E, 3, 2, 1);
impl_to_spi_miso!(PortName::E, 18, 5, 0);
impl_to_spi_miso!(PortName::E, 19, 2, 0);
impl_to_spi_miso!(PortName::A, 16, 5, 0);
impl_to_spi_miso!(PortName::A, 17, 2, 0);
impl_to_spi_miso!(PortName::B, 16, 5, 1);
impl_to_spi_miso!(PortName::B, 17, 2, 1);
impl_to_spi_miso!(PortName::C, 6, 5, 0);
impl_to_spi_miso!(PortName::C, 7, 2, 0);
impl_to_spi_miso!(PortName::D, 2, 5, 0);
impl_to_spi_miso!(PortName::D, 3, 2, 0);
impl_to_spi_miso!(PortName::D, 6, 5, 1);
impl_to_spi_miso!(PortName::D, 7, 2, 1);

pub struct SpiSck<'a, const N: PortName, const P: usize> {
    spi: u8,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> SpiSck<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.spi
    }
}

pub trait ToSpiSck<'a, const N: PortName, const P: usize> {
    fn to_spi_sck(self) -> SpiSck<'a, N, P>;
}

macro_rules! impl_to_spi_sck {
    ($pn:expr, $pp:expr, $mode:expr, $bus:expr) => {
        impl<'a> ToSpiSck<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_spi_sck(mut self) -> SpiSck<'a, $pn, $pp> {
                self.set_mode($mode);

                SpiSck {
                    spi: $bus,
                    _pin: self,
                }
            }
        }
    };
}

impl_to_spi_sck!(PortName::E, 2, 2, 1);
impl_to_spi_sck!(PortName::E, 17, 2, 0);
impl_to_spi_sck!(PortName::A, 15, 2, 0);
impl_to_spi_sck!(PortName::B, 9, 2, 1);
impl_to_spi_sck!(PortName::B, 11, 2, 1);
impl_to_spi_sck!(PortName::C, 5, 2, 0);
impl_to_spi_sck!(PortName::D, 1, 2, 0);
impl_to_spi_sck!(PortName::D, 5, 2, 1);

pub struct SpiCs<'a, const N: PortName, const P: usize> {
    spi: u8,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> SpiCs<'a, N, P> {
    pub fn bus(&self) -> u8 {
        self.spi
    }
}

pub trait ToSpiCs<'a, const N: PortName, const P: usize> {
    fn to_spi_cs(self) -> SpiCs<'a, N, P>;
}

macro_rules! impl_to_spi_cs {
    ($pn:expr, $pp:expr, $mode:expr, $bus:expr) => {
        impl<'a> ToSpiCs<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_spi_cs(mut self) -> SpiCs<'a, $pn, $pp> {
                self.set_mode($mode);

                SpiCs {
                    spi: $bus,
                    _pin: self,
                }
            }
        }
    };
}

impl_to_spi_cs!(PortName::E, 4, 2, 1);
impl_to_spi_cs!(PortName::E, 16, 2, 0);
impl_to_spi_cs!(PortName::A, 14, 2, 0);
impl_to_spi_cs!(PortName::B, 8, 2, 1);
impl_to_spi_cs!(PortName::B, 10, 2, 1);
impl_to_spi_cs!(PortName::C, 4, 2, 0);
impl_to_spi_cs!(PortName::D, 0, 2, 0);
impl_to_spi_cs!(PortName::D, 4, 2, 1);

pub struct TpmPin<'a, const N: PortName, const P: usize> {
    tpm: TpmNum,
    ch: ChannelNum,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> TpmPin<'a, N, P> {
    pub fn tpm(&self) -> TpmNum {
        self.tpm
    }

    pub fn ch(&self) -> ChannelNum {
        self.ch
    }
}

pub trait ToTpmPin<'a, const N: PortName, const P: usize> {
    fn to_tpm(self) -> TpmPin<'a, N, P>;
}

macro_rules! impl_to_tpm {
    ($pn:expr, $pp:expr, $mode:expr, $tpm:expr, $ch:expr) => {
        impl<'a> ToTpmPin<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_tpm(mut self) -> TpmPin<'a, $pn, $pp> {
                self.set_mode($mode);

                TpmPin {
                    tpm: $tpm,
                    ch: $ch,
                    _pin: self,
                }
            }
        }
    };
}

// e20:3-1,0
impl_to_tpm!(PortName::E, 20, 3, TpmNum::TPM1, ChannelNum::Ch0);
// e21:3-1,1
impl_to_tpm!(PortName::E, 21, 3, TpmNum::TPM1, ChannelNum::Ch1);
// e22:3-2,0
impl_to_tpm!(PortName::E, 22, 3, TpmNum::TPM2, ChannelNum::Ch0);
// e23:3-2,1
impl_to_tpm!(PortName::E, 23, 3, TpmNum::TPM2, ChannelNum::Ch1);
// e29:3-0,2
impl_to_tpm!(PortName::E, 29, 3, TpmNum::TPM0, ChannelNum::Ch2);
// e30:3-0,3
impl_to_tpm!(PortName::E, 30, 3, TpmNum::TPM0, ChannelNum::Ch3);
// e31:3-0,4
impl_to_tpm!(PortName::E, 31, 3, TpmNum::TPM0, ChannelNum::Ch4);
// e24:3-0,0
impl_to_tpm!(PortName::E, 24, 3, TpmNum::TPM0, ChannelNum::Ch0);
// e25:3-0,1
impl_to_tpm!(PortName::E, 25, 3, TpmNum::TPM0, ChannelNum::Ch1);
// e26:3-0,5
impl_to_tpm!(PortName::E, 26, 3, TpmNum::TPM0, ChannelNum::Ch5);
// a0:3-0,5
impl_to_tpm!(PortName::A, 0, 3, TpmNum::TPM0, ChannelNum::Ch5);
// a1:3-2,0
impl_to_tpm!(PortName::A, 1, 3, TpmNum::TPM2, ChannelNum::Ch0);
// a2:3-2,1
impl_to_tpm!(PortName::A, 2, 3, TpmNum::TPM2, ChannelNum::Ch1);
// a3:3-0,0
impl_to_tpm!(PortName::A, 3, 3, TpmNum::TPM0, ChannelNum::Ch0);
// a4:3-0,1
impl_to_tpm!(PortName::A, 4, 3, TpmNum::TPM0, ChannelNum::Ch1);
// a5:3-0,2
impl_to_tpm!(PortName::A, 5, 3, TpmNum::TPM0, ChannelNum::Ch2);
// a6:3-0,3
impl_to_tpm!(PortName::A, 6, 3, TpmNum::TPM0, ChannelNum::Ch3);
// a7:3-0,4
impl_to_tpm!(PortName::A, 7, 3, TpmNum::TPM0, ChannelNum::Ch4);
// a12:3-1,0
impl_to_tpm!(PortName::A, 12, 3, TpmNum::TPM1, ChannelNum::Ch0);
// a13:3-1,1
impl_to_tpm!(PortName::A, 13, 3, TpmNum::TPM1, ChannelNum::Ch1);
// b0:3-1,0
impl_to_tpm!(PortName::B, 0, 3, TpmNum::TPM1, ChannelNum::Ch0);
// b1:3-1,1
impl_to_tpm!(PortName::B, 1, 3, TpmNum::TPM1, ChannelNum::Ch1);
// b2:3-2,0
impl_to_tpm!(PortName::B, 2, 3, TpmNum::TPM2, ChannelNum::Ch0);
// b3:3-2,1
impl_to_tpm!(PortName::B, 3, 3, TpmNum::TPM2, ChannelNum::Ch1);
// b18:3-2,0
impl_to_tpm!(PortName::B, 18, 3, TpmNum::TPM2, ChannelNum::Ch0);
// b19:3-2,1
impl_to_tpm!(PortName::B, 19, 3, TpmNum::TPM2, ChannelNum::Ch1);
// c1:4-0,0
impl_to_tpm!(PortName::C, 1, 4, TpmNum::TPM0, ChannelNum::Ch0);
// c2:4-0,1
impl_to_tpm!(PortName::C, 2, 4, TpmNum::TPM0, ChannelNum::Ch1);
// c3:4-0,2
impl_to_tpm!(PortName::C, 3, 4, TpmNum::TPM0, ChannelNum::Ch2);
// c4:4-0,3
impl_to_tpm!(PortName::C, 4, 4, TpmNum::TPM0, ChannelNum::Ch3);
// c8:3-0,4
impl_to_tpm!(PortName::C, 8, 3, TpmNum::TPM0, ChannelNum::Ch4);
// c9:3-0,5
impl_to_tpm!(PortName::C, 9, 3, TpmNum::TPM0, ChannelNum::Ch5);
// d0:4-0,0
impl_to_tpm!(PortName::D, 0, 4, TpmNum::TPM0, ChannelNum::Ch0);
// d1:4-0,1
impl_to_tpm!(PortName::D, 1, 4, TpmNum::TPM0, ChannelNum::Ch1);
// d2:4-0,2
impl_to_tpm!(PortName::D, 2, 4, TpmNum::TPM0, ChannelNum::Ch2);
// d3:4-0,3
impl_to_tpm!(PortName::D, 3, 4, TpmNum::TPM0, ChannelNum::Ch3);
// d4:4-0,4
impl_to_tpm!(PortName::D, 4, 4, TpmNum::TPM0, ChannelNum::Ch4);
// d5:4-0,5
impl_to_tpm!(PortName::D, 5, 4, TpmNum::TPM0, ChannelNum::Ch5);

pub struct UartRx<'a, const N: PortName, const P: usize> {
    uart: UartNum,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> UartRx<'a, N, P> {
    pub fn bus(&self) -> UartNum {
        self.uart
    }
}

pub trait ToUartRx<'a, const N: PortName, const P: usize> {
    fn to_uart_rx(self) -> UartRx<'a, N, P>;
}

macro_rules! impl_to_uart_rx {
    ($pn:expr, $pp:expr, $mode:expr, $bus:expr) => {
        impl<'a> ToUartRx<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_uart_rx(mut self) -> UartRx<'a, $pn, $pp> {
                self.set_mode($mode);

                UartRx {
                    uart: $bus,
                    _pin: self,
                }
            }
        }
    };
}

impl_to_uart_rx!(PortName::E, 1, 3, UartNum::UART1);
impl_to_uart_rx!(PortName::E, 17, 3, UartNum::UART2);
impl_to_uart_rx!(PortName::E, 21, 4, UartNum::UART0);
impl_to_uart_rx!(PortName::E, 23, 4, UartNum::UART2);
impl_to_uart_rx!(PortName::A, 1, 2, UartNum::UART0);
impl_to_uart_rx!(PortName::A, 15, 3, UartNum::UART0);
impl_to_uart_rx!(PortName::A, 18, 3, UartNum::UART1);
impl_to_uart_rx!(PortName::B, 16, 3, UartNum::UART0);
impl_to_uart_rx!(PortName::C, 3, 3, UartNum::UART1);
impl_to_uart_rx!(PortName::D, 2, 3, UartNum::UART2);
impl_to_uart_rx!(PortName::D, 4, 3, UartNum::UART2);
impl_to_uart_rx!(PortName::D, 6, 3, UartNum::UART0);

pub struct UartTx<'a, const N: PortName, const P: usize> {
    uart: UartNum,
    _pin: Pin<'a, N, P>,
}

impl<'a, const N: PortName, const P: usize> UartTx<'a, N, P> {
    pub fn bus(&self) -> UartNum {
        self.uart
    }
}

pub trait ToUartTx<'a, const N: PortName, const P: usize> {
    fn to_uart_tx(self) -> UartTx<'a, N, P>;
}

macro_rules! impl_to_uart_tx {
    ($pn:expr, $pp:expr, $mode:expr, $bus:expr) => {
        impl<'a> ToUartTx<'a, $pn, $pp> for Pin<'a, $pn, $pp> {
            fn to_uart_tx(mut self) -> UartTx<'a, $pn, $pp> {
                self.set_mode($mode);

                UartTx {
                    uart: $bus,
                    _pin: self,
                }
            }
        }
    };
}

impl_to_uart_tx!(PortName::E, 0, 3, UartNum::UART1);
impl_to_uart_tx!(PortName::E, 16, 3, UartNum::UART2);
impl_to_uart_tx!(PortName::E, 20, 4, UartNum::UART0);
impl_to_uart_tx!(PortName::E, 22, 4, UartNum::UART2);
impl_to_uart_tx!(PortName::A, 2, 2, UartNum::UART0);
impl_to_uart_tx!(PortName::A, 14, 3, UartNum::UART0);
impl_to_uart_tx!(PortName::A, 19, 3, UartNum::UART1);
impl_to_uart_tx!(PortName::B, 17, 3, UartNum::UART0);
impl_to_uart_tx!(PortName::C, 4, 3, UartNum::UART1);
impl_to_uart_tx!(PortName::D, 3, 3, UartNum::UART2);
impl_to_uart_tx!(PortName::D, 5, 3, UartNum::UART2);
impl_to_uart_tx!(PortName::D, 7, 3, UartNum::UART0);

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
