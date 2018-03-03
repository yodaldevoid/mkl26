use bit_field::BitField;
use cortex_m::peripheral::NVIC;
use volatile_register::{RO,RW};

use adc::{Adc,AdcDiff};
use i2c::{I2cMaster,OpMode};
#[cfg(feature = "i2c-slave")]
use i2c::{Address,I2cSlave};
use port::{AdcPin,AdcDiffPPin,AdcDiffMPin,I2cSda,I2cScl,Port,PortName,UartRx,UartTx};
use uart::Uart;

pub struct ClockGate {
    gate: &'static mut RW<u32>,
    bit: u8
}

impl ClockGate {
    fn new(reg: usize, bit: usize) -> ClockGate {
        assert!(reg <= 7);
        assert!(bit <= 31);
        // TODO: use BME in the absence of bitbanding
        let base: usize = 0x40048028;
        let reg_offset = 4 * (reg - 1);
        let ptr = (base + reg_offset) as *mut RW<u32>;
        unsafe {
            ClockGate { gate: &mut *ptr, bit: bit as u8 }
        }
    }

    fn enable(&mut self) {
        unsafe {
            self.gate.modify(|mut gate| {
                gate.set_bit(self.bit, true);
                gate
            });
        }
    }

    fn disable(&mut self) {
        unsafe {
            self.gate.modify(|mut gate| {
                gate.set_bit(self.bit, false);
                gate
            });
        }
    }

    fn is_enabled(&self) -> bool {
        self.gate.read().get_bit(self.bit)
    }
}

impl Drop for ClockGate {
    fn drop(&mut self) {
        self.disable();
    }
}

#[repr(C,packed)]
struct SimRegs {
    sopt1:      RW<u32>,
    sopt1_cfg:  RW<u32>,
    _pad0:      [u32; 1023],
    sopt2:      RW<u32>,
    _pad1:      u32,
    sopt4:      RW<u32>,
    sopt5:      RW<u32>,
    _pad2:      u32,
    sopt7:      RW<u32>,
    _pad3:      [u32; 2],
    sdid:       RO<u32>,
    _pad4:      [u32; 3],
    scgc4:      RW<u32>,
    scgc5:      RW<u32>,
    scgc6:      RW<u32>,
    scgc7:      RW<u32>,
    clkdiv1:    RW<u32>,
    _pad5:      u32,
    fcfg1:      RO<u32>,
    fcfg2:      RO<u32>,
    _pad6:      u32,
    uidmh:      RO<u32>,
    uidml:      RO<u32>,
    uidl:       RO<u32>,
}

pub struct Sim {
    reg: &'static mut SimRegs
}

// TODO: fake atomic
static mut SIM_INIT: bool = false;

impl Sim {
    pub fn new() -> Sim {
        // TODO: fake atomic
        let was_init = unsafe { SIM_INIT };
        unsafe { SIM_INIT = true; }
        if was_init {
            panic!("Cannot initialize SIM: It's already active");
        }
        let reg = unsafe { &mut *(0x40047000 as *mut SimRegs) };
        Sim { reg: reg }
    }

    pub fn set_dividers(&mut self, core: u32, bus_flash: u32) {
        let mut clkdiv: u32 = 0;
        clkdiv.set_bits(28..32, core - 1);
        clkdiv.set_bits(16..18, bus_flash - 1);
        unsafe { self.reg.clkdiv1.write(clkdiv); }
    }

    pub fn port(&mut self, port: PortName) -> Port {
        let mut gate = match port {
            PortName::A => ClockGate::new(5,9),
            PortName::B => ClockGate::new(5,10),
            PortName::C => ClockGate::new(5,11),
            PortName::D => ClockGate::new(5,12),
            PortName::E => ClockGate::new(5,13),
        };
        if gate.is_enabled() {
            panic!("Cannot create Port instance; it is already in use");
        }
        gate.enable();
        unsafe {
            Port::new(port, gate)
        }
    }

    pub fn uart<'a, 'b,
                R: Into<Option<UartRx<'a>>>,
                T: Into<Option<UartTx<'b>>>>(&mut self,
                                             uart: u8,
                                             rx: R,
                                             tx: T,
                                             clkdiv: (u16,u8),
                                             rxfifo: bool,
                                             txfifo: bool)
                                             -> Result<Uart<'a, 'b>, ()> {
        let mut gate = match uart {
            0 => ClockGate::new(4, 10),
            1 => ClockGate::new(4, 11),
            2 => ClockGate::new(4, 12),
            _ => return Err(()) //panic!("Cannot enable clock for UART {}", uart)
        };
        if gate.is_enabled() {
            return Err(()) //panic!("Cannot create Uart instance; it is already in use");
        }
        gate.enable();
        unsafe {
            Uart::new(uart, rx.into(), tx.into(), clkdiv, rxfifo, txfifo, gate)
        }
    }

    // TODO: support higher bus numbers for other chips
    pub fn i2c_master<'a, 'b>(&mut self,
                              scl: I2cScl<'a>,
                              sda: I2cSda<'b>,
                              nvic: &mut NVIC,
                              clkdiv: (u8, u8),
                              op_mode: OpMode)
                              -> Result<I2cMaster<'a, 'b>, ()> {
        if scl.bus() != sda.bus() {
            return Err(());
        }

        let mut gate = match scl.bus() {
            0 => ClockGate::new(4, 6),
            1 => ClockGate::new(4, 7),
            _ => return Err(()) //panic!("Cannot enable clock for I2C {}", bus)
        };
        if gate.is_enabled() {
            return Err(()) //panic!("Cannot create I2c instance; it is already in use");
        }
        gate.enable();
        unsafe {
            I2cMaster::new(scl, sda, nvic, clkdiv, op_mode, gate)
        }
    }

    // TODO: support higher bus numbers for other chips
    #[cfg(feature = "i2c-slave")]
    pub fn i2c_slave<'a, 'b>(&mut self,
                             scl: I2cScl<'a>,
                             sda: I2cSda<'b>,
                             nvic: &mut NVIC,
                             addr: Address,
                             general_call: bool)
                             -> Result<I2cSlave<'a, 'b>, ()> {
        if scl.bus() != sda.bus() {
            return Err(());
        }

        let mut gate = match scl.bus() {
            0 => ClockGate::new(4, 6),
            1 => ClockGate::new(4, 7),
            _ => return Err(()) //panic!("Cannot enable clock for I2C {}", bus)
        };
        if gate.is_enabled() {
            return Err(()) //panic!("Cannot create I2c instance; it is already in use");
        }
        gate.enable();
        unsafe {
            I2cSlave::new(scl, sda, nvic, addr, general_call, gate)
        }
    }

    pub fn adc<'a, P: Into<Option<AdcPin<'a>>>>(&mut self,
                                                adc: u8,
                                                ch: u8,
                                                mode: u8,
                                                clkdiv: u8,
                                                pin: P)
                                                -> Result<Adc<'a>, ()> {
        let mut gate = match adc {
            0 => ClockGate::new(6, 27),
            _ => return Err(()) //panic!("Cannot enable clock for UART {}", uart)
        };
        if gate.is_enabled() {
            return Err(()) //panic!("Cannot create Uart instance; it is already in use");
        }
        gate.enable();
        unsafe {
            Adc::new(adc, ch, mode, clkdiv, pin.into(), gate)
        }
    }

    pub fn adc_diff<'a, 'b>(&mut self,
                            adc: u8,
                            ch: u8,
                            mode: u8,
                            clkdiv: u8,
                            pos: Option<AdcDiffPPin<'a>>,
                            neg: Option<AdcDiffMPin<'b>>)
                            -> Result<AdcDiff<'a, 'b>, ()> {
        let mut gate = match adc {
            0 => ClockGate::new(6, 27),
            _ => return Err(()) //panic!("Cannot enable clock for UART {}", uart)
        };
        if gate.is_enabled() {
            return Err(()) //panic!("Cannot create Uart instance; it is already in use");
        }
        gate.enable();
        unsafe {
            AdcDiff::new(adc, ch, mode, clkdiv, pos, neg, gate)
        }
    }
}

impl Drop for Sim {
    fn drop(&mut self) {
        // TODO: fake atomic
        unsafe { SIM_INIT = false; }
    }
}

pub mod cop {
    use bit_field::BitField;
    use volatile_register::{RW,WO};

    pub enum Timeout {
        /// COP timeout after 2^5 LPO cycles or 2^13 bus clock cycles.
        Short = 1,
        /// COP timeout after 2^8 LPO cycles or 2^16 bus clock cycles.
        Medium = 2,
        /// COP timeout after 2^10 LPO cycles or 2^18 bus clock cycles.
        Long = 3
    }

    pub enum ClockSource {
        /// Internal 1 kHZ clock is the source to COP
        Internal,
        /// Bus clock is the source to COP.
        Bus(Windowing)
    }

    pub enum Windowing {
        Normal = 0,
        Windowed = 1
    }

    struct CopRegs {
        copc:   RW<u32>,
        srvcop: WO<u32>
    }

    pub struct Cop {
        reg: &'static mut CopRegs
    }

    impl Cop {
        pub fn new() -> Cop {
            let reg = unsafe { &mut *(0x4004_8100 as *mut CopRegs) };
            Cop { reg }
        }

        /// Initializes the COP with the given settings.
        ///
        /// `None` disables the COP.
        ///
        /// Once called the first time, this function effectively does nothing as
        /// the COP control register can only be written once.
        pub unsafe fn init(&mut self, settings: Option<(Timeout, ClockSource)>) {
            fn map_settings((timeout, source): (Timeout, ClockSource)) -> u32 {
                let mut copc = 0;
                copc.set_bits(2..4, timeout as u32);
                if let ClockSource::Bus(windowing) = source {
                    copc.set_bit(1, true);
                    copc.set_bit(0, windowing as u32 == 1);
                }
                copc
            }

            self.reg.copc.write(settings.map_or(0, map_settings));
        }

        /// Resets the COP timout counter.
        #[inline(always)]
        pub unsafe fn reset(&mut self) {
            self.reg.srvcop.write(0x55);
            self.reg.srvcop.write(0xAA);
        }
    }
}
