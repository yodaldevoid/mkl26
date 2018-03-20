//! FIFO is 8 bytes big

use core::marker::PhantomData;

use bit_field::BitField;
use ux::{u3, u4};
use volatile_register::{RO, RW};

use port::{Gpio, SpiMosi, SpiMiso, SpiSck, SpiCs};
use sim::ClockGate;

#[repr(C,packed)]
struct SpiRegs {
    s:  RO<u8>,
    br: RW<u8>,
    c2: RW<u8>,
    c1: RW<u8>,
    ml: RW<u8>,
    mh: RW<u8>,
    dl: RW<u8>,
    dh: RW<u8>,
    ci: RW<u8>,
    c3: RW<u8>,
}

pub struct SpiMaster<'a, 'b, 'c, 'd, W: Word> {
    reg: &'static mut SpiRegs,
    _mosi: Option<SpiMosi<'a>>,
    _miso: Option<SpiMiso<'b>>,
    _sck: SpiSck<'c>,
    _cs: Option<SpiCs<'d>>,
    _gate: ClockGate,
    _char: PhantomData<W>,
    op_mode: OpMode,
}

pub struct SpiSlave<'a, 'b, 'c, 'd, W: Word> {
    reg: &'static mut SpiRegs,
    _mosi: Option<SpiMosi<'a>>,
    _miso: Option<SpiMiso<'b>>,
    _sck: SpiSck<'c>,
    _cs: Option<SpiCs<'d>>,
    _gate: ClockGate,
    _char: PhantomData<W>,
}

#[derive(PartialEq)]
pub enum OpMode {
    /// Immediate mode - All transmits and receives are blocking. Doesn't use ISR.
    IMM,
    #[cfg(feature = "spi-isr")]
    /// Interrupt mode - Transmits and receives are non-blocking. Uses ISR.
    ISR,
    #[cfg(feature = "spi-dma")]
    /// DMA mode - Like interrupt mode, but it uses DMA to do the magic
    DMA
}

#[derive(PartialEq)]
pub enum Polarity {
    IdleLow,
    IdleHigh,
}

#[derive(PartialEq)]
pub enum Phase {
    CaptureOnFirstTransition,
    CaptureOnSecondTransition,
}

pub trait Word {
    fn size() -> usize;
}

impl Word for u8 {
    fn size() -> usize {
        8
    }
}

impl Word for u16 {
    fn size() -> usize {
        16
    }
}

//pub fn calc_clkdiv(baud: u32, clock_freq: u32) -> (u3, u4) {
//    let clkdiv = (clock_freq / baud) as u8;
//    clkdiv = (sppr + 1) x 2^(spr + 1);
//}

// TODO: single wire mode
// TODO: low power mode3
// TODO: DMA Rx
// TODO: DMA Tx
// TODO: MSb/LSb
// TODO: hardware match
// TODO: mode fault feature
impl<'a, 'b, 'c, 'd, W: Word> SpiMaster<'a, 'b, 'c, 'd, W> {
    pub(crate) unsafe fn new(
        mosi: Option<SpiMosi<'a>>,
        miso: Option<SpiMiso<'b>>,
        sck: SpiSck<'c>,
        cs: Option<SpiCs<'d>>,
        clkdiv: (u3, u4),
        op_mode: OpMode,
        polarity: Polarity,
        phase: Phase,
        fifo: bool,
        gate: ClockGate,
    ) -> Result<SpiMaster<'a, 'b, 'c, 'd, W>, ()> {
        let bus = sck.bus();

        if let Some(mosi) = mosi.as_ref() {
            if mosi.bus() != bus {
                return Err(());
            }
        }
        if let Some(miso) = miso.as_ref() {
            if miso.bus() != bus {
                return Err(());
            }
        }
        if let Some(cs) = cs.as_ref() {
            if cs.bus() != bus {
                return Err(());
            }
        }
        if clkdiv.0 > u3::new(7) || clkdiv.1 > u4::new(8) {
            return Err(());
        }

        let reg = match bus {
            0 => &mut *(0x4007_6000 as *mut SpiRegs),
            1 => &mut *(0x4007_7000 as *mut SpiRegs),
            _ => unreachable!()
        };

        let mut c1 = 0;
        c1.set_bit(7, op_mode != OpMode::IMM); // SPRF and MODF *or* Rx FIFO Full interrupt enable
        c1.set_bit(6, true); // SPI enable
        c1.set_bit(5, op_mode != OpMode::IMM); // SPTEF or Tx FIFO empty interrupt enable
        c1.set_bit(4, true); // slave/master :: 0/1
        c1.set_bit(3, polarity == Polarity::IdleHigh); // clock polarity idle low/high :: 0/1
        c1.set_bit(2, phase == Phase::CaptureOnFirstTransition); // clock phase middle/start :: 0/1
        c1.set_bit(1, true); // Slave Select Output Enable
        c1.set_bit(0, false); // most/least significant bit :: 0/1
        reg.c1.write(c1);

        let mut c2 = 0;
        c2.set_bit(7, false); // match enable
        c2.set_bit(6, W::size() == 16); // 8/16 bits :: 0/1
        #[cfg(feature = "spi-dma")]
        c2.set_bit(5, op_mode == OpMode::DMA); // Tx DMA enable
        c2.set_bit(4, cs.is_some()); // Master Mode-Fault enable
        c2.set_bit(3, false); // Bidirectional mode input/output select
        #[cfg(feature = "spi-dma")]
        c2.set_bit(2, op_mode == OpMode::DMA); // Rx DMA enable
        c2.set_bit(1, false); // SPI stop in wait mode
        c2.set_bit(0, false); // SPI Bidirectional (single wire) mode enable
        reg.c2.write(c2);

        let mut c3 = 0;
        c3.set_bit(3, true); // FIFO interrpt flag clearing method
        c3.set_bit(2, fifo); // Tx FIFO nearly empty interrupt enable
        c3.set_bit(1, fifo); // Rx FIFO nearly full interrupt enable
        c3.set_bit(0, fifo); // FIFO mode enable
        reg.c3.write(c3);

        // BR
        let mut br: u8 = 0;
        br.set_bits(4..7, clkdiv.0.into());
        br.set_bits(0..4, clkdiv.1.into());
        reg.br.write(br);

        // MH/ML
        // set if hardware match enabled

        Ok(SpiMaster {
            reg,
            _mosi: mosi,
            _miso: miso,
            _sck: sck,
            _cs: cs,
            _gate: gate,
            _char: PhantomData,
            op_mode,
        })
    }
}

impl<'a, 'b, 'c, 'd> SpiMaster<'a, 'b, 'c, 'd, u8> {
    pub fn send(&mut self, word: u8) -> Result<(), ()> {
        if self.op_mode == OpMode::IMM {
            if self.reg.s.read().get_bit(5) {
                unsafe {
                    self.reg.dl.write(word);
                }
                Ok(())
            } else {
                Err(())
            }
        } else {
            unimplemented!()
        }
    }

    pub fn read(&mut self) -> Result<u8, ()> {
        if self.op_mode == OpMode::IMM {
            if self.reg.s.read().get_bit(7) {
                Ok(self.reg.dl.read())
            } else {
                Err(())
            }
        } else {
            unimplemented!()
        }
    }

    // TODO: check match
}

impl<'a, 'b, 'c, 'd> SpiMaster<'a, 'b, 'c, 'd, u16> {
    pub fn send(&mut self, word: u16) -> Result<(), ()> {
        if self.op_mode == OpMode::IMM {
            if self.reg.s.read().get_bit(5) {
                unsafe {
                    self.reg.dh.write((word >> 8) as u8);
                    self.reg.dl.write(word as u8);
                }
                Ok(())
            } else {
                Err(())
            }
        } else {
            unimplemented!()
        }
    }

    pub fn read(&mut self) -> Result<u16, ()> {
        if self.op_mode == OpMode::IMM {
            if self.reg.s.read().get_bit(7) {
                let d = ((self.reg.dh.read() as u16) << 8) | (self.reg.dl.read() as u16);
                Ok(d)
            } else {
                Err(())
            }
        } else {
            unimplemented!()
        }
    }

    // TODO: check match
}
