use volatile::Volatile;
use bit_field::BitField;

pub enum PortName {
    A,
    B,
    C,
    D,
    E
}

#[repr(C,packed)]
pub struct Port {
    pcr:        [Volatile<u32>; 32],
    gpclr:      Volatile<u32>,
    gpchr:      Volatile<u32>,
    reserved_0: [u8; 24],
    isfr:       Volatile<u32>
}

impl Port {
    pub unsafe fn new(name: PortName) -> &'static mut Port {
        &mut * match name {
            PortName::A => 0x40049000 as *mut Port,
            PortName::B => 0x4004A000 as *mut Port,
            PortName::C => 0x4004B000 as *mut Port,
            PortName::D => 0x4004C000 as *mut Port,
            PortName::E => 0x4004D000 as *mut Port
        }
    }
}

pub struct Pin {
    port: *mut Port,
    pin: usize
}

impl Port {
    pub unsafe fn pin(&mut self, p: usize) -> Pin {
        Pin { port: self, pin: p }
    }
}

impl Pin {
    pub fn set_mode(&mut self, mode: u32) {
        let port = unsafe { &mut *self.port };
        port.pcr[self.pin].update(|pcr| {
            pcr.set_bits(8..11, mode);
        });
    }
}

#[repr(C,packed)]
struct GpioBitband {
    pdor: [Volatile<u32>; 32],
    psor: [Volatile<u32>; 32],
    pcor: [Volatile<u32>; 32],
    ptor: [Volatile<u32>; 32],
    pdir: [Volatile<u32>; 32],
    pddr: [Volatile<u32>; 32]
}

pub struct Gpio {
    gpio: *mut GpioBitband,
    pin: usize
}

impl Port {
    pub fn name(&self) -> PortName {
        let addr = (self as *const Port) as u32;
        match addr {
            0x40049000 => PortName::A,
            0x4004A000 => PortName::B,
            0x4004B000 => PortName::C,
            0x4004C000 => PortName::D,
            0x4004D000 => PortName::E,
            _ => unreachable!()
        }
    }
}

impl Pin {
    pub fn to_gpio(mut self) -> Gpio {
        unsafe {
            self.set_mode(1);
            let port = &mut *self.port;
            Gpio::new(port.name(), self.pin)
        }
    }
}

impl Gpio {
    pub unsafe fn new(port: PortName, pin: usize) -> Gpio {
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
            (&mut (*self.gpio)).pddr[self.pin].write(1);
        }
    }

    pub fn high(&mut self) {
        unsafe {
            (&mut (*self.gpio)).psor[self.pin].write(1);
        }
    }
}
