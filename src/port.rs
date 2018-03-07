use core::cell::UnsafeCell;

use bit_field::BitField;
use volatile_register::{RO,RW,WO};

use atomic::InterruptAtomic;
use sim::ClockGate;

pub enum PortName {
    A,
    B,
    C,
    D,
    E
}

#[repr(C,packed)]
struct PortRegs {
    pcr:        [RW<u32>; 32],
    gpclr:      WO<u32>,
    gpchr:      WO<u32>,
    reserved_0: [u8; 24],
    isfr:       RW<u32>
}

pub struct Port {
    reg: UnsafeCell<&'static mut PortRegs>,
    locks: [InterruptAtomic<bool>; 32],
    _gate: ClockGate
}

impl Port {
    pub unsafe fn new(name: PortName, gate: ClockGate) -> Port {
        let reg = &mut * match name {
            PortName::A => 0x40049000 as *mut PortRegs,
            PortName::B => 0x4004A000 as *mut PortRegs,
            PortName::C => 0x4004B000 as *mut PortRegs,
            PortName::D => 0x4004C000 as *mut PortRegs,
            PortName::E => 0x4004D000 as *mut PortRegs
        };

        Port { reg: UnsafeCell::new(reg), locks: Default::default(), _gate: gate }
    }

    pub fn name(&self) -> PortName {
        let addr = (self.reg() as *const PortRegs) as u32;
        match addr {
            0x40049000 => PortName::A,
            0x4004A000 => PortName::B,
            0x4004B000 => PortName::C,
            0x4004C000 => PortName::D,
            0x4004D000 => PortName::E,
            _ => unreachable!()
        }
    }

    pub fn pin(&self, p: usize) -> Pin {
        assert!(p < 32);
        let was_init = self.locks[p].swap(true);
        if was_init {
            panic!("Pin {} is already in use", p);
        }
        Pin { port: self, pin: p }
    }

    unsafe fn drop_pin(&self, p: usize) {
        assert!(p < 32);
        self.locks[p].store(false)
    }

    fn reg(&self) -> &'static mut PortRegs {
        // NOTE: This does no validation. It's on the calling
        // functions to ensure they're not accessing the same
        // registers from multiple codepaths. If they can't make those
        // guarantees, they should be marked as `unsafe` (See
        // `set_pin_mode` as an example).
        unsafe {
            *self.reg.get()
        }
    }
}

pub struct Pin<'a> {
    port: &'a Port,
    pin: usize
}

#[derive(PartialEq)]
pub enum DriveStrength {
    Low = 0,
    High = 1,
}

#[derive(PartialEq)]
pub enum SlewRate {
    Fast = 0,
    Slow = 1,
}

pub enum Pull {
    Disable = 0,
    Down = 2,
    Up = 3
}

impl<'a> Pin<'a> {
    fn set_mode(&mut self, mode: u32) {
        unsafe {
            self.port.reg().pcr[self.pin].modify(|mut pcr| {
                pcr.set_bits(8..11, mode);
                pcr
            });
        }
    }

    fn drive_strength(&mut self, strength: DriveStrength) {
        unsafe {
            self.port.reg().pcr[self.pin].modify(|mut pcr| {
                pcr.set_bit(6, strength == DriveStrength::High);
                pcr
            });
        }
    }

    fn open_drain(&mut self, enable: bool) {
        unsafe {
            self.port.reg().pcr[self.pin].modify(|mut pcr| {
                pcr.set_bit(5, enable);
                pcr
            });
        }
    }

    fn slew_rate(&mut self, rate: SlewRate) {
        unsafe {
            self.port.reg().pcr[self.pin].modify(|mut pcr| {
                pcr.set_bit(2, rate == SlewRate::Slow);
                pcr
            });
        }
    }

    fn pull(&mut self, pull: Pull) {
        unsafe {
            self.port.reg().pcr[self.pin].modify(|mut pcr| {
                pcr.set_bits(0..2, pull as u32);
                pcr
            });
        }
    }

    pub fn to_gpio(mut self) -> Gpio<'a> {
        unsafe {
            self.set_mode(1);
            Gpio::new(self.port.name(), self)
        }
    }

    pub fn to_adc(mut self) -> Result<AdcPin<'a>, ()> {
        match (self.port.name(), self.pin) {
            //  9-e20:0-S0,0
            (PortName::E, 20) |
            // 10-e21:0-S0,4a
            (PortName::E, 21) |
            // 11-e22:0-S0,3
            (PortName::E, 22) |
            // 12-e23:0-S0,7a
            (PortName::E, 23) |
            // 17-e29:0-S0,4b
            (PortName::E, 29) |
            // 18-e30:0-S0,23
            (PortName::E, 30) |
            // 35-b0:0-S0,8
            (PortName::B, 0) |
            // 36-b1:0-S0,9
            (PortName::B, 1) |
            // 37-b2:0-S0,12
            (PortName::B, 2) |
            // 38-b3:0-S0,13
            (PortName::B, 3) |
            // 43-c0:0-S0,14
            (PortName::C, 0) |
            // 44-c1:0-S0,15
            (PortName::C, 1) |
            // 45-c2:0-S0,11
            (PortName::C, 2) |
            // 58-d1:0-S0,5b
            (PortName::D, 1) |
            // 62-d5:0-S0,6b
            (PortName::D, 5) |
            // 58-d6:0-S0,7b
            (PortName::D, 6) => {
                self.set_mode(0);
                Ok(AdcPin { _pin: self })
            },
            _ => Err(())
        }
    }

    pub fn to_adc_diff_p(mut self) -> Result<AdcDiffPPin<'a>, ()> {
        match (self.port.name(), self.pin) {
            //  9-e20:0-D0,0P
            (PortName::E, 20) |
            // 11-e22:0-D0,3P
            (PortName::E, 22) => {
                self.set_mode(0);
                Ok(AdcDiffPPin { _pin: self })
            }
            _ => Err(())
        }
    }

    pub fn to_adc_diff_m(mut self) -> Result<AdcDiffMPin<'a>, ()> {
        match (self.port.name(), self.pin) {
            // 10-e21:0-D0,0M
            (PortName::E, 21) |
            // 12-e23:0-D0,3M
            (PortName::E, 23) => {
                self.set_mode(0);
                Ok(AdcDiffMPin { _pin: self })
            }
            _ => Err(())
        }
    }

    pub fn to_i2c_scl(mut self, pullup_ext: bool) -> Result<I2cScl<'a>, ()> {
        let bus = match (self.port.name(), self.pin) {
            (PortName::E, 1) => {
                self.set_mode(6);
                1
            },
            (PortName::E, 24) => {
                self.set_mode(5);
                0
            },
            (PortName::A, 3) => {
                self.set_mode(2);
                1
            },
            (PortName::B, 0) => {
                self.set_mode(2);
                0
            },
            (PortName::B, 2) => {
                self.set_mode(2);
                0
            },
            (PortName::C, 1) => {
                self.set_mode(2);
                1
            },
            (PortName::C, 8) => {
                self.set_mode(2);
                0
            },
            (PortName::C, 10) => {
                self.set_mode(2);
                1
            },
            _ => return Err(())
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

        Ok(I2cScl { i2c: bus, _pin: self })
    }

    pub fn to_i2c_sda(mut self, pullup_ext: bool) -> Result<I2cSda<'a>, ()> {
        let bus = match (self.port.name(), self.pin) {
            (PortName::E, 0) => {
                self.set_mode(6);
                1
            },
            (PortName::E, 25) => {
                self.set_mode(5);
                0
            },
            (PortName::A, 4) => {
                self.set_mode(2);
                1
            },
            (PortName::B, 1) => {
                self.set_mode(2);
                0
            },
            (PortName::B, 3) => {
                self.set_mode(2);
                0
            },
            (PortName::C, 2) => {
                self.set_mode(2);
                1
            },
            (PortName::C, 9) => {
                self.set_mode(2);
                0
            },
            (PortName::C, 11) => {
                self.set_mode(2);
                1
            },
            _ => return Err(())
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

        Ok(I2cSda { i2c: bus, _pin: self })
    }

    pub fn to_uart_rx(mut self) -> Result<UartRx<'a>, ()> {
        match (self.port.name(), self.pin) {
            (PortName::E, 1) => {
                self.set_mode(3);
                Ok(UartRx { uart: 1, _pin: self })
            },
            (PortName::E, 17) => {
                self.set_mode(3);
                Ok(UartRx { uart: 2, _pin: self })
            },
            (PortName::E, 21) => {
                self.set_mode(4);
                Ok(UartRx { uart: 0, _pin: self })
            },
            (PortName::E, 23) => {
                self.set_mode(4);
                Ok(UartRx { uart: 2, _pin: self })
            },
            (PortName::A, 1) => {
                self.set_mode(2);
                Ok(UartRx { uart: 0, _pin: self })
            },
            (PortName::A, 15) => {
                self.set_mode(3);
                Ok(UartRx { uart: 0, _pin: self })
            },
            (PortName::A, 18) => {
                self.set_mode(3);
                Ok(UartRx { uart: 1, _pin: self })
            },
            (PortName::B, 16) => {
                self.set_mode(3);
                Ok(UartRx { uart: 0, _pin: self })
            },
            (PortName::C, 3) => {
                self.set_mode(3);
                Ok(UartRx { uart: 1, _pin: self })
            },
            (PortName::D, 2) => {
                self.set_mode(3);
                Ok(UartRx { uart: 2, _pin: self })
            },
            (PortName::D, 4) => {
                self.set_mode(3);
                Ok(UartRx { uart: 2, _pin: self })
            },
            (PortName::D, 6) => {
                self.set_mode(3);
                Ok(UartRx { uart: 0, _pin: self })
            },
            _ => Err(())
        }
    }

    pub fn to_uart_tx(mut self) -> Result<UartTx<'a>, ()> {
        match (self.port.name(), self.pin) {
            (PortName::E, 0) => {
                self.set_mode(3);
                Ok(UartTx { uart: 1, _pin: self })
            },
            (PortName::E, 16) => {
                self.set_mode(3);
                Ok(UartTx { uart: 2, _pin: self })
            },
            (PortName::E, 20) => {
                self.set_mode(4);
                Ok(UartTx { uart: 0, _pin: self })
            },
            (PortName::E, 22) => {
                self.set_mode(4);
                Ok(UartTx { uart: 2, _pin: self })
            },
            (PortName::A, 2) => {
                self.set_mode(2);
                Ok(UartTx { uart: 0, _pin: self })
            },
            (PortName::A, 14) => {
                self.set_mode(3);
                Ok(UartTx { uart: 0, _pin: self })
            },
            (PortName::A, 19) => {
                self.set_mode(3);
                Ok(UartTx { uart: 1, _pin: self })
            },
            (PortName::B, 17) => {
                self.set_mode(3);
                Ok(UartTx { uart: 0, _pin: self })
            },
            (PortName::C, 4) => {
                self.set_mode(3);
                Ok(UartTx { uart: 1, _pin: self })
            },
            (PortName::D, 3) => {
                self.set_mode(3);
                Ok(UartTx { uart: 2, _pin: self })
            },
            (PortName::D, 5) => {
                self.set_mode(3);
                Ok(UartTx { uart: 2, _pin: self })
            },
            (PortName::D, 7) => {
                self.set_mode(3);
                Ok(UartTx { uart: 0, _pin: self })
            },
            _ => Err(())
        }
    }
}

impl <'a> Drop for Pin<'a> {
    fn drop(&mut self) {
        unsafe {
            self.port.drop_pin(self.pin);
        }
    }
}

#[repr(C,packed)]
struct GpioReg {
    pdor: RW<u32>,
    psor: WO<u32>,
    pcor: WO<u32>,
    ptor: WO<u32>,
    pdir: RO<u32>,
    pddr: RW<u32>
}


// TODO: maybe split into input and output
pub struct Gpio<'a> {
    gpio: *mut GpioReg,
    pin: Pin<'a>
}

impl<'a> Gpio<'a> {
    // TODO: BME?
    pub unsafe fn new(port: PortName, pin: Pin<'a>) -> Gpio<'a> {
        let gpio = match port {
            // GPIO
            #[cfg(not(feature = "fgpio"))]
            PortName::A => 0x400FF000 as *mut GpioReg,
            #[cfg(not(feature = "fgpio"))]
            PortName::B => 0x400FF040 as *mut GpioReg,
            #[cfg(not(feature = "fgpio"))]
            PortName::C => 0x400FF080 as *mut GpioReg,
            #[cfg(not(feature = "fgpio"))]
            PortName::D => 0x400FF0C0 as *mut GpioReg,
            #[cfg(not(feature = "fgpio"))]
            PortName::E => 0x400FF100 as *mut GpioReg,
            // FGPIO
            #[cfg(feature = "fgpio")]
            PortName::A => 0xF8000000 as *mut GpioReg,
            #[cfg(feature = "fgpio")]
            PortName::B => 0xF8000040 as *mut GpioReg,
            #[cfg(feature = "fgpio")]
            PortName::C => 0xF8000080 as *mut GpioReg,
            #[cfg(feature = "fgpio")]
            PortName::D => 0xF80000C0 as *mut GpioReg,
            #[cfg(feature = "fgpio")]
            PortName::E => 0xF8000100 as *mut GpioReg,
        };

        Gpio { gpio: gpio, pin: pin }
    }

    pub fn input(&mut self) {
        unsafe {
            (&mut (*self.gpio)).pddr.modify(|mut pddr| {
                pddr &= !(1 << self.pin.pin);
                pddr
            });
        }
    }

    pub fn output(&mut self) {
        unsafe {
            (&mut (*self.gpio)).pddr.modify(|mut pddr| {
                pddr |= 1 << self.pin.pin;
                pddr
            });
        }
    }

    pub fn high(&mut self) {
        unsafe {
            (&mut (*self.gpio)).psor.write(1 << self.pin.pin);
        }
    }

    pub fn low(&mut self) {
        unsafe {
            (&mut (*self.gpio)).pcor.write(1 << self.pin.pin);
        }
    }

    pub fn toggle(&mut self) {
        unsafe {
            (&mut (*self.gpio)).ptor.write(1 << self.pin.pin);
        }
    }
}

pub struct AdcPin<'a> {
    _pin: Pin<'a>
}

impl<'a> AdcPin<'a> {
    pub fn port_name(&self) -> PortName {
        self._pin.port.name()
    }

    pub fn pin(&self) -> usize {
        self._pin.pin
    }
}

pub struct AdcDiffPPin<'a> {
    _pin: Pin<'a>
}

impl<'a> AdcDiffPPin<'a> {
    pub fn port_name(&self) -> PortName {
        self._pin.port.name()
    }

    pub fn pin(&self) -> usize {
        self._pin.pin
    }
}

pub struct AdcDiffMPin<'a> {
    _pin: Pin<'a>
}

impl<'a> AdcDiffMPin<'a> {
    pub fn port_name(&self) -> PortName {
        self._pin.port.name()
    }

    pub fn pin(&self) -> usize {
        self._pin.pin
    }
}

pub struct I2cScl<'a> {
    i2c: u8,
    _pin: Pin<'a>
}

pub struct I2cSda<'a> {
    i2c: u8,
    _pin: Pin<'a>
}

impl<'a> I2cScl<'a> {
    pub fn bus(&self) -> u8 {
        self.i2c
    }
}

impl<'a> I2cSda<'a> {
    pub fn bus(&self) -> u8 {
        self.i2c
    }
}

pub struct UartRx<'a> {
    uart: u8,
    _pin: Pin<'a>
}

pub struct UartTx<'a> {
    uart: u8,
    _pin: Pin<'a>
}

impl<'a> UartRx<'a> {
    pub fn bus(&self) -> u8 {
        self.uart
    }
}

impl<'a> UartTx<'a> {
    pub fn bus(&self) -> u8 {
        self.uart
    }
}
