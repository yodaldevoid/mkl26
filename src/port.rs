use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool,Ordering};

use volatile_register::{RO,RW,WO};
use bit_field::BitField;

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
    locks: [AtomicBool; 32],
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
        let was_init = self.locks[p].swap(true, Ordering::Relaxed);
        if was_init {
            panic!("Pin {} is already in use", p);
        }
        Pin { port: self, pin: p }
    }

    unsafe fn drop_pin(&self, p: usize) {
        assert!(p < 32);
        self.locks[p].store(false, Ordering::Relaxed)
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

impl<'a> Pin<'a> {
    fn set_mode(&mut self, mode: u32) {
        unsafe {
            self.port.reg().pcr[self.pin].modify(|mut pcr| {
                pcr.set_bits(8..11, mode);
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
            // 45-c2:0-S0,4b
            (PortName::C, 2) |
            // 58-d1:0-S0,5b
            (PortName::D, 1) |
            // 62-d5:0-S0,6b
            (PortName::D, 5) |
            // 63-d6:0-S0,7b
            (PortName::D, 6) |
            // 35-b0:0-S0,8
            // 35-b0:0-S1,8
            (PortName::B, 0) |
            // 36-b1:0-S0,9
            // 36-b1:0-S1,9
            (PortName::B, 1) |
            // 37-b2:0-S0,12
            (PortName::B, 2) |
            // 38-b3:0-S0,13
            (PortName::B, 3) |
            // 43-c0:0-S0,14
            (PortName::C, 0) |
            // 44-c1:0-S0,15
            (PortName::C, 1) |

            //  1-e0:0-S1,4a
            (PortName::E, 0) |
            // 53-c8:0-S1,4b
            (PortName::C, 8) |
            //  2-e1:0-S1,5a
            (PortName::E, 1) |
            // 54-c9:0-S1,5b
            (PortName::C, 9) |
            // 55-c10:0-S1,6b
            (PortName::C, 10) |
            // 56-c11:0-S1,7b
            (PortName::C, 11) => {
                self.set_mode(0);
                Ok(AdcPin { _pin: self })
            },
            _ => Err(())
        }
    }

    pub fn to_uart_rx(mut self) -> Result<UartRx<'a>, ()> {
        match (self.port.name(), self.pin) {
            (PortName::E, 1) => {
                self.set_mode(3);
                Ok(UartRx { uart: 1, _pin: self })
            },
            (PortName::A, 1) => {
                self.set_mode(2);
                Ok(UartRx { uart: 0, _pin: self })
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
            (PortName::A, 2) => {
                self.set_mode(2);
                Ok(UartTx { uart: 0, _pin: self })
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
struct GpioBitband {
    pdor: [RW<u32>; 32],
    psor: [WO<u32>; 32],
    pcor: [WO<u32>; 32],
    ptor: [WO<u32>; 32],
    pdir: [RO<u32>; 32],
    pddr: [RW<u32>; 32]
}

pub struct Gpio<'a> {
    gpio: *mut GpioBitband,
    pin: Pin<'a>
}

impl<'a> Gpio<'a> {
    pub unsafe fn new(port: PortName, pin: Pin<'a>) -> Gpio<'a> {
        let gpio = match port {
            PortName::A => 0x43FE0000 as *mut GpioBitband,
            PortName::B => 0x43FE0800 as *mut GpioBitband,
            PortName::C => 0x43FE1000 as *mut GpioBitband,
            PortName::D => 0x43FE1800 as *mut GpioBitband,
            PortName::E => 0x43FE2000 as *mut GpioBitband
        };

        Gpio { gpio: gpio, pin: pin }
    }

    pub fn output(&mut self) {
        unsafe {
            (&mut (*self.gpio)).pddr[self.pin.pin].write(1);
        }
    }

    pub fn high(&mut self) {
        unsafe {
            (&mut (*self.gpio)).psor[self.pin.pin].write(1);
        }
    }

    pub fn low(&mut self) {
        unsafe {
            (&mut (*self.gpio)).pcor[self.pin.pin].write(1);
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

pub struct UartRx<'a> {
    uart: u8,
    _pin: Pin<'a>
}

pub struct UartTx<'a> {
    uart: u8,
    _pin: Pin<'a>
}

impl<'a> UartRx<'a> {
    pub fn uart(&self) -> u8 {
        self.uart
    }
}

impl<'a> UartTx<'a> {
    pub fn uart(&self) -> u8 {
        self.uart
    }
}
