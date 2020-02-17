use bit_field::BitField;
use embedded_hal::spi::Mode;
use volatile_register::{RO, RW};

use crate::adc::{self, Adc};
use crate::atomic::{BmeAtomic, InterruptAtomic};
use crate::i2c::{self, Divider, I2cMaster, Multiplier};
#[cfg(feature = "i2c-slave")]
use crate::i2c::{Address, I2cSlave};
use crate::pit::Pit;
use crate::port::{I2cScl, I2cSda};
use crate::port::{Port, PortName};
use crate::port::{SpiCs, SpiMiso, SpiMosi, SpiSck};
use crate::port::{UartRx, UartTx};
use crate::spi::{self, Divisor, SpiMaster};
use crate::tpm::{self, ClockMode, PwmSelect, TpmNum, TpmPeriodic, TpmSingleShot};
use crate::uart::{ConnMode, Uart, UartNum};

pub struct ClockGate {
    gate: &'static mut BmeAtomic<u32>,
    bit: u8, // TODO: replace with a const generic when that comes around?
}

impl ClockGate {
    fn new(reg: usize, bit: usize) -> ClockGate {
        assert!(reg <= 7);
        assert!(bit <= 31);
        let base: usize = 0x4004_8028;
        let reg_offset = 4 * (reg - 1);
        let ptr = (base + reg_offset) as *mut BmeAtomic<u32>;
        unsafe {
            ClockGate {
                gate: &mut *ptr,
                bit: bit as u8,
            }
        }
    }

    fn enable(&mut self) {
        unsafe {
            self.gate.or(*0.set_bit(self.bit, true));
        }
    }

    fn disable(&mut self) {
        unsafe {
            self.gate.and(*u32::max_value().set_bit(self.bit, false));
        }
    }

    fn is_enabled(&self) -> bool {
        unsafe { self.gate.ubfx(self.bit as usize, 1) != 0 }
    }
}

impl Drop for ClockGate {
    fn drop(&mut self) {
        self.disable();
    }
}

/// MCGPLLCLK/MCGFLLCLK clock source
///
/// Selects the MCGPLLCLK or MCGFLLCLK clock for various peripheral clocking options.
pub enum PllFllSel {
    /// FLL
    Fll = 0,
    /// PLL divided by 2
    PllDiv2 = 1,
}

/// Clock source
///
/// UART0 transmit and receive clock selection and TPM clock selection use this.
pub enum ClkSrc {
    /// Transmit and receive clock disabled
    Disabled = 0,
    /// MCG FLL output or MCG PLL output divided by 2. Which determined by PLLFLLSEL.
    McgXLL = 1,
    /// System oscillator from OSCCLK
    OscER = 2,
    /// MCG internal reference clock
    McgIR = 3,
}

const SIM_ADDR: usize = 0x4004_7000;

#[repr(C, packed)]
struct SimRegs {
    sopt1: RW<u32>,
    sopt1_cfg: RW<u32>,
    _pad0: [u32; 1023],
    sopt2: RW<u32>,
    _pad1: u32,
    sopt4: RW<u32>,
    sopt5: RW<u32>,
    _pad2: u32,
    sopt7: RW<u32>,
    _pad3: [u32; 2],
    sdid: RO<u32>,
    _pad4: [u32; 3],
    scgc4: RW<u32>,
    scgc5: RW<u32>,
    scgc6: RW<u32>,
    scgc7: RW<u32>,
    clkdiv1: RW<u32>,
    _pad5: u32,
    fcfg1: RW<u32>,
    fcfg2: RO<u32>,
    _pad6: u32,
    uidmh: RO<u32>,
    uidml: RO<u32>,
    uidl: RO<u32>,
}

pub struct Sim {
    reg: &'static mut SimRegs,
}

static SIM_INIT: InterruptAtomic<bool> = InterruptAtomic::new(false);

impl Sim {
    pub fn new() -> Sim {
        let was_init = SIM_INIT.swap(true);
        if was_init {
            panic!("Cannot initialize SIM: It's already active");
        }
        let reg = unsafe { &mut *(SIM_ADDR as *mut SimRegs) };
        Sim { reg }
    }

    pub fn set_dividers(&mut self, core: u32, bus_flash: u32) {
        let mut clkdiv: u32 = 0;
        clkdiv.set_bits(28..32, core - 1);
        clkdiv.set_bits(16..18, bus_flash - 1);
        unsafe {
            self.reg.clkdiv1.write(clkdiv);
        }
    }

    /// Select which clock is output on the MCGPLLCLK/MCGFLLCLK clock source
    pub unsafe fn select_pll_fll(&mut self, sel: PllFllSel) {
        self.reg.sopt2.modify(|mut sopt2| {
            sopt2.set_bit(16, sel as u8 == 1);
            sopt2
        });
    }

    pub fn port<const N: PortName>(&mut self) -> Port<N> {
        let mut gate = match N {
            PortName::A => ClockGate::new(5, 9),
            PortName::B => ClockGate::new(5, 10),
            PortName::C => ClockGate::new(5, 11),
            PortName::D => ClockGate::new(5, 12),
            PortName::E => ClockGate::new(5, 13),
        };
        if gate.is_enabled() {
            panic!("Cannot create Port instance; it is already in use");
        }
        gate.enable();
        unsafe { Port::new(gate) }
    }

    pub fn uart<'a, 'b, R, T, const NR: PortName, const NT: PortName, const PT: usize, const PR: usize>(
        &mut self,
        uart: UartNum,
        rx: R,
        tx: T,
        clkdiv: u16,
    ) -> Result<Uart<'a, 'b, u8, NR, NT, PR, PT>, ()>
    where
        R: Into<Option<UartRx<'a, NR, PR>>>,
        T: Into<Option<UartTx<'b, NT, PT>>>
    {
        let mut gate = match uart {
            UartNum::UART0 => ClockGate::new(4, 10),
            UartNum::UART1 => ClockGate::new(4, 11),
            UartNum::UART2 => ClockGate::new(4, 12),
        };
        if gate.is_enabled() {
            return Err(());
        }
        gate.enable();
        unsafe { Uart::new(uart, rx.into(), tx.into(), clkdiv, ConnMode::TwoWire, gate) }
    }

    // TODO: fix the false const generics
    pub fn uart_loopback<'a, 'b>(
        &mut self,
        uart: UartNum,
        clkdiv: u16,
    ) -> Result<Uart<'a, 'b, u8, {PortName::A}, {PortName::A}, {0}, {0}>, ()> {
        let mut gate = match uart {
            UartNum::UART0 => ClockGate::new(4, 10),
            UartNum::UART1 => ClockGate::new(4, 11),
            UartNum::UART2 => ClockGate::new(4, 12),
        };
        if gate.is_enabled() {
            return Err(());
        }
        gate.enable();
        unsafe { Uart::new(uart, None, None, clkdiv, ConnMode::Loop, gate) }
    }

    pub unsafe fn set_tpm_clksrc(&mut self, clksrc: ClkSrc) {
        self.reg.sopt2.modify(|mut sopt2| {
            sopt2.set_bits(24..25, clksrc as u32);
            sopt2
        });
    }

    pub unsafe fn set_uart0_clksrc(&mut self, clksrc: ClkSrc) {
        self.reg.sopt2.modify(|mut sopt2| {
            sopt2.set_bits(26..28, clksrc as u32);
            sopt2
        });
    }

    pub fn i2c_master<'a, 'b, const NC: PortName, const ND: PortName, const PC: usize, const PD: usize>(
        &mut self,
        scl: I2cScl<'a, NC, PC>,
        sda: I2cSda<'b, ND, PD>,
        clkdiv: (Multiplier, Divider),
        op_mode: i2c::OpMode,
    ) -> Result<I2cMaster<'a, 'b, NC, ND, PC, PD>, ()> {
        if scl.bus() != sda.bus() {
            return Err(());
        }

        let mut gate = match scl.bus() {
            0 => ClockGate::new(4, 6),
            1 => ClockGate::new(4, 7),
            _ => return Err(()),
        };
        if gate.is_enabled() {
            return Err(());
        }
        gate.enable();
        unsafe { I2cMaster::new(scl, sda, clkdiv, op_mode, gate) }
    }

    #[cfg(feature = "i2c-slave")]
    pub fn i2c_slave<'a, 'b, const NC: PortName, const ND: PortName, const PC: usize, const PD: usize>(
        &mut self,
        scl: I2cScl<'a, NC, PC>,
        sda: I2cSda<'b, ND, PD>,
        addr: Address,
        general_call: bool,
    ) -> Result<I2cSlave<'a, 'b, NC, ND, PC, PD>, ()> {
        let mut gate = match scl.bus() {
            0 => ClockGate::new(4, 6),
            1 => ClockGate::new(4, 7),
            _ => return Err(()),
        };
        if gate.is_enabled() {
            return Err(());
        }
        gate.enable();
        unsafe { I2cSlave::new(scl, sda, addr, general_call, gate) }
    }

    pub fn spi_master<
        'a,
        'b,
        'c,
        'd,
        O,
        I,
        S,
        W,
        const NO: PortName,
        const NI: PortName,
        const NC: PortName,
        const NS: PortName,
        const PO: usize,
        const PI: usize,
        const PC: usize,
        const PS: usize,
    >(
        &mut self,
        mosi: O,
        miso: I,
        sck: SpiSck<'c, NC, PC>,
        cs: S,
        clkdiv: (spi::Prescale, Divisor),
        op_mode: spi::OpMode,
        Mode { polarity, phase }: Mode,
        fifo: bool,
    ) -> Result<SpiMaster<'a, 'b, 'c, 'd, W, NO, NI, NC, NS, PO, PI, PC, PS>, ()>
    where
        O: Into<Option<SpiMosi<'a, NO, PO>>>,
        I: Into<Option<SpiMiso<'b, NI, PI>>>,
        S: Into<Option<SpiCs<'d, NS, PS>>>,
        W: spi::Word,
    {
        let mut gate = match sck.bus() {
            0 => ClockGate::new(4, 22),
            1 => ClockGate::new(4, 23),
            _ => return Err(()),
        };
        if gate.is_enabled() {
            return Err(());
        }
        gate.enable();

        unsafe {
            SpiMaster::new(
                mosi.into(),
                miso.into(),
                sck,
                cs.into(),
                clkdiv,
                op_mode,
                polarity,
                phase,
                fifo,
                gate,
            )
        }
    }

    pub fn tpm_periodic(
        &mut self,
        name: TpmNum,
        cpwms: PwmSelect,
        cmod: ClockMode,
        clkdivider: tpm::Prescale,
        interrupt: bool,
        count: u16,
    ) -> Result<TpmPeriodic, ()> {
        let mut gate = match name {
            TpmNum::TPM0 => ClockGate::new(6, 24),
            TpmNum::TPM1 => ClockGate::new(6, 25),
            TpmNum::TPM2 => ClockGate::new(6, 26),
        };
        if gate.is_enabled() {
            return Err(());
        }
        gate.enable();

        unsafe {
            Ok(TpmPeriodic::new(
                name, cpwms, cmod, clkdivider, interrupt, count, gate,
            ))
        }
    }

    pub fn tpm_single_shot(
        &mut self,
        name: TpmNum,
        cpwms: PwmSelect,
        cmod: ClockMode,
        clkdivider: tpm::Prescale,
        interrupt: bool,
        count: u16,
    ) -> Result<TpmSingleShot, ()> {
        let mut gate = match name {
            TpmNum::TPM0 => ClockGate::new(6, 24),
            TpmNum::TPM1 => ClockGate::new(6, 25),
            TpmNum::TPM2 => ClockGate::new(6, 26),
        };
        if gate.is_enabled() {
            return Err(());
        }
        gate.enable();

        unsafe {
            Ok(TpmSingleShot::new(
                name, cpwms, cmod, clkdivider, interrupt, count, gate,
            ))
        }
    }

    pub fn pit(&mut self) -> Result<Pit, ()> {
        let mut gate = ClockGate::new(6, 23);
        if gate.is_enabled() {
            return Err(());
        }
        gate.enable();

        unsafe { Ok(Pit::new(gate)) }
    }

    pub fn adc(
        &mut self,
        adc: u8,
        resolution: adc::Resolution,
        clkdiv: adc::Divisor,
        vref: adc::VoltageRef,
    ) -> Result<Adc, ()> {
        let mut gate = match adc {
            0 => ClockGate::new(6, 27),
            _ => return Err(()),
        };
        if gate.is_enabled() {
            return Err(());
        }
        gate.enable();
        unsafe { Adc::new(adc, resolution, clkdiv, vref, gate) }
    }
}

impl Drop for Sim {
    fn drop(&mut self) {
        SIM_INIT.store(false);
    }
}

pub mod cop {
    use bit_field::BitField;
    use volatile_register::{RW, WO};

    pub enum Timeout {
        /// COP timeout after 2^5 LPO cycles or 2^13 bus clock cycles.
        Short = 1,
        /// COP timeout after 2^8 LPO cycles or 2^16 bus clock cycles.
        Medium = 2,
        /// COP timeout after 2^10 LPO cycles or 2^18 bus clock cycles.
        Long = 3,
    }

    pub enum ClockSource {
        /// Internal 1 kHZ clock is the source to COP
        Internal,
        /// Bus clock is the source to COP.
        Bus(Windowing),
    }

    pub enum Windowing {
        Normal = 0,
        Windowed = 1,
    }

    const COP_ADDR: usize = 0x4004_8100;

    struct CopRegs {
        copc: RW<u32>,
        srvcop: WO<u32>,
    }

    pub struct Cop {
        reg: &'static mut CopRegs,
    }

    impl Cop {
        pub fn new() -> Cop {
            let reg = unsafe { &mut *(COP_ADDR as *mut CopRegs) };
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

    #[cfg(test)]
    mod tests {
        use super::*;

        #[cfg_attr(rustfmt, rustfmt_skip)]
        #[test]
        fn cop_test() {
            unsafe {
                let reg = &*(COP_ADDR as *const CopRegs);
                assert_eq!(0x4004_8100 as *const RW<u32>, &reg.copc     as *const RW<u32>, "copc");
                assert_eq!(0x4004_8104 as *const WO<u32>, &reg.srvcop   as *const WO<u32>, "srvcop");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn sim_test() {
        unsafe {
            let reg = &*(SIM_ADDR as *const SimRegs);
            assert_eq!(0x4004_7000 as *const RW<u32>, &reg.sopt1        as *const RW<u32>, "sopt1");
            assert_eq!(0x4004_7004 as *const RW<u32>, &reg.sopt1_cfg    as *const RW<u32>, "sopt1_cfg");
            assert_eq!(0x4004_8004 as *const RW<u32>, &reg.sopt2        as *const RW<u32>, "sopt2");
            assert_eq!(0x4004_800C as *const RW<u32>, &reg.sopt4        as *const RW<u32>, "sopt4");
            assert_eq!(0x4004_8010 as *const RW<u32>, &reg.sopt5        as *const RW<u32>, "sopt5");
            assert_eq!(0x4004_8018 as *const RW<u32>, &reg.sopt7        as *const RW<u32>, "sopt7");
            assert_eq!(0x4004_8024 as *const RO<u32>, &reg.sdid         as *const RO<u32>, "sdid");
            assert_eq!(0x4004_8034 as *const RW<u32>, &reg.scgc4        as *const RW<u32>, "scgc4");
            assert_eq!(0x4004_8038 as *const RW<u32>, &reg.scgc5        as *const RW<u32>, "scgc5");
            assert_eq!(0x4004_803C as *const RW<u32>, &reg.scgc6        as *const RW<u32>, "scgc6");
            assert_eq!(0x4004_8040 as *const RW<u32>, &reg.scgc7        as *const RW<u32>, "scgc7");
            assert_eq!(0x4004_8044 as *const RW<u32>, &reg.clkdiv1      as *const RW<u32>, "clkdiv1");
            assert_eq!(0x4004_804C as *const RW<u32>, &reg.fcfg1        as *const RW<u32>, "fcfg1");
            assert_eq!(0x4004_8050 as *const RO<u32>, &reg.fcfg2        as *const RO<u32>, "fcfg2");
            assert_eq!(0x4004_8058 as *const RO<u32>, &reg.uidmh        as *const RO<u32>, "uidmh");
            assert_eq!(0x4004_805C as *const RO<u32>, &reg.uidml        as *const RO<u32>, "uidml");
            assert_eq!(0x4004_8060 as *const RO<u32>, &reg.uidl         as *const RO<u32>, "uidl");
        }
    }
}
