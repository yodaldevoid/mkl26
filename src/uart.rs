use core::fmt::{self, Write};
use core::marker::PhantomData;

use bit_field::BitField;
use volatile_register::{RO, RW};

use port::{UartRx, UartTx};
use sim::ClockGate;

const UART0_ADDR: usize = 0x4006_A000;
const UART1_ADDR: usize = 0x4006_B000;
const UART2_ADDR: usize = 0x4006_C000;

#[repr(C, packed)]
struct UartRegs {
    bdh:  RW<u8>,
    bdl:  RW<u8>,
    c1:   RW<u8>,
    c2:   RW<u8>,
    s1:   RO<u8>,
    s2:   RW<u8>,
    c3:   RW<u8>,
    d:    RW<u8>,
    diff: UartDiff,
}

#[repr(C)]
union UartDiff {
    u0: Uart0Diff,
    u1: Uart12Diff,
}

#[repr(C, packed)]
struct Uart0Diff {
    ma1: RW<u8>,
    ma2: RW<u8>,
    c4:  RW<u8>,
    c5:  RW<u8>,
}

#[repr(C, packed)]
struct Uart12Diff {
    c4: RW<u8>,
}

/// A UART peripheral
///
/// B is the "character" size used for the UART. This can be 8, 9, or 10 bits.
// TODO: consider grabbing crate ux
pub struct Uart<'a, 'b, B> {
    reg:   &'static mut UartRegs,
    _rx:   Option<UartRx<'a>>,
    _tx:   Option<UartTx<'b>>,
    _gate: ClockGate,
    _bus:  u8, // TODO: replace with a const generic when that comes around
    _char: PhantomData<B>,
}

#[derive(Clone, Copy)]
pub(crate) enum ConnMode {
    TwoWire,
    Loop,
}

/// clock_freq - Frequency of module clock in Hz. UART0 uses the clock specified by
/// SIM_SOPT2[UART0SRC]. UART1 and UART2 use the bus clock.
pub fn calc_clkdiv(baud: u32, clock_freq: u32) -> u16 {
    ((((clock_freq / 16) + (baud / 2)) / baud) & 0xFFF) as u16
}

// TODO: flow control
// TODO: support ISR mode
// TODO: support DMA mode
// TODO: 9 bit mode
// TODO: 10 bit mode
// TODO: stop bits
impl<'a, 'b> Uart<'a, 'b, u8> {
    pub(crate) unsafe fn new(
        bus: u8,
        rx: Option<UartRx<'a>>,
        tx: Option<UartTx<'b>>,
        clkdiv: u16,
        conn_mode: ConnMode,
        gate: ClockGate,
    ) -> Result<Uart<'a, 'b, u8>, ()> {
        if let Some(r) = rx.as_ref() {
            if r.bus() != bus {
                return Err(());
            }
        }
        if let Some(t) = tx.as_ref() {
            if t.bus() != bus {
                return Err(());
            }
        }
        if clkdiv > 0xFFF {
            return Err(());
        }

        let reg = match bus {
            0 => &mut *(UART0_ADDR as *mut UartRegs),
            1 => &mut *(UART1_ADDR as *mut UartRegs),
            2 => &mut *(UART2_ADDR as *mut UartRegs),
            _ => unreachable!(),
        };

        reg.bdh.modify(|mut bdh| {
            bdh.set_bits(0..4, clkdiv.get_bits(8..12) as u8);
            bdh
        });
        reg.bdl.write(clkdiv.get_bits(0..8) as u8);

        reg.c1.modify(|mut c1| {
            match conn_mode {
                ConnMode::TwoWire => {
                    c1.set_bit(7, false);
                }
                ConnMode::Loop => {
                    c1.set_bit(7, true);
                    c1.set_bit(5, false);
                }
            }
            c1
        });

        reg.c2.modify(|mut c2| {
            match conn_mode {
                ConnMode::TwoWire => {
                    c2.set_bit(2, rx.is_some());
                    c2.set_bit(3, tx.is_some());
                }
                ConnMode::Loop => {
                    c2.set_bits(2..4, 0x3);
                }
            }
            c2
        });

        Ok(Uart {
            reg,
            _tx: tx,
            _rx: rx,
            _gate: gate,
            _bus: bus,
            _char: PhantomData,
        })
    }

    /// Read is non-blocking
    pub fn read_byte(&mut self) -> Result<u8, ()> {
        // wait for something in the rx buffer
        if self.reg.s1.read().get_bit(5) {
            Ok(self.reg.d.read())
        } else {
            Err(())
        }
    }

    /// Write is non-blocking
    pub fn write_byte(&mut self, b: u8) -> Result<(), ()> {
        // wait for tx buffer to not be full
        if self.reg.s1.read().get_bit(7) {
            unsafe {
                self.reg.d.write(b);
            }
            Ok(())
        } else {
            Err(())
        }
    }

    // TODO: add timeout
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        if buf.is_empty() {
            return Ok(0);
        }

        let mut index: usize = 0;

        while index < buf.len() - 1 {
            if let Ok(b) = self.read_byte() {
                buf[index] = b;
                index += 1;
            }
        }

        Ok(buf.len())
    }

    // TODO: add timeout
    pub fn write(&mut self, buf: &[u8]) -> Result<usize, ()> {
        if buf.is_empty() {
            return Ok(0);
        }

        let mut index: usize = 0;

        while index < buf.len() - 1 {
            if let Ok(()) = self.write_byte(buf[index]) {
                index += 1;
            }
        }

        Ok(buf.len())
    }
}

impl<'a, 'b> Write for Uart<'a, 'b, u8> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            // Retry if the buffer is full
            while let Err(()) = self.write_byte(b) {}
        }
        Ok(())
    }
}

impl<'a, 'b, B> Uart<'a, 'b, B> {
    /// Flush is non-blocking
    pub fn flush_byte(&self) -> Result<(), ()> {
        if self.reg.s1.read().get_bit(6) {
            Ok(())
        } else {
            Err(())
        }
    }
}

impl<'a, 'b, B> Drop for Uart<'a, 'b, B> {
    fn drop(&mut self) {
        unsafe {
            self.reg.c2.modify(|mut c2| {
                c2.set_bits(2..4, 0);
                c2
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn uart0_test() {
        unsafe {
            let reg = &*(UART0_ADDR as *const UartRegs);
            assert_eq!(0x4006_A000 as *const RW<u8>, &reg.bdh   as *const RW<u8>, "bdh");
            assert_eq!(0x4006_A001 as *const RW<u8>, &reg.bdl   as *const RW<u8>, "bdl");
            assert_eq!(0x4006_A002 as *const RW<u8>, &reg.c1    as *const RW<u8>, "c1");
            assert_eq!(0x4006_A003 as *const RW<u8>, &reg.c2    as *const RW<u8>, "c2");
            assert_eq!(0x4006_A004 as *const RO<u8>, &reg.s1    as *const RO<u8>, "s1");
            assert_eq!(0x4006_A005 as *const RW<u8>, &reg.s2    as *const RW<u8>, "s2");
            assert_eq!(0x4006_A006 as *const RW<u8>, &reg.c3    as *const RW<u8>, "c3");
            assert_eq!(0x4006_A007 as *const RW<u8>, &reg.d     as *const RW<u8>, "d");
            assert_eq!(0x4006_A008 as *const RW<u8>, &reg.diff.u0.ma1   as *const RW<u8>, "ma1");
            assert_eq!(0x4006_A009 as *const RW<u8>, &reg.diff.u0.ma2   as *const RW<u8>, "ma2");
            assert_eq!(0x4006_A00A as *const RW<u8>, &reg.diff.u0.c4    as *const RW<u8>, "c4");
            assert_eq!(0x4006_A00B as *const RW<u8>, &reg.diff.u0.c5    as *const RW<u8>, "c5");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn uart1_test() {
        unsafe {
            let reg = &*(UART1_ADDR as *const UartRegs);
            assert_eq!(0x4006_B000 as *const RW<u8>, &reg.bdh   as *const RW<u8>, "bdh");
            assert_eq!(0x4006_B001 as *const RW<u8>, &reg.bdl   as *const RW<u8>, "bdl");
            assert_eq!(0x4006_B002 as *const RW<u8>, &reg.c1    as *const RW<u8>, "c1");
            assert_eq!(0x4006_B003 as *const RW<u8>, &reg.c2    as *const RW<u8>, "c2");
            assert_eq!(0x4006_B004 as *const RO<u8>, &reg.s1    as *const RO<u8>, "s1");
            assert_eq!(0x4006_B005 as *const RW<u8>, &reg.s2    as *const RW<u8>, "s2");
            assert_eq!(0x4006_B006 as *const RW<u8>, &reg.c3    as *const RW<u8>, "c3");
            assert_eq!(0x4006_B007 as *const RW<u8>, &reg.d     as *const RW<u8>, "d");
            assert_eq!(0x4006_B008 as *const RW<u8>, &reg.diff.u1.c4    as *const RW<u8>, "c4");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn uart2_test() {
        unsafe {
            let reg = &*(UART2_ADDR as *const UartRegs);
            assert_eq!(0x4006_C000 as *const RW<u8>, &reg.bdh   as *const RW<u8>, "bdh");
            assert_eq!(0x4006_C001 as *const RW<u8>, &reg.bdl   as *const RW<u8>, "bdl");
            assert_eq!(0x4006_C002 as *const RW<u8>, &reg.c1    as *const RW<u8>, "c1");
            assert_eq!(0x4006_C003 as *const RW<u8>, &reg.c2    as *const RW<u8>, "c2");
            assert_eq!(0x4006_C004 as *const RO<u8>, &reg.s1    as *const RO<u8>, "s1");
            assert_eq!(0x4006_C005 as *const RW<u8>, &reg.s2    as *const RW<u8>, "s2");
            assert_eq!(0x4006_C006 as *const RW<u8>, &reg.c3    as *const RW<u8>, "c3");
            assert_eq!(0x4006_C007 as *const RW<u8>, &reg.d     as *const RW<u8>, "d");
            assert_eq!(0x4006_C008 as *const RW<u8>, &reg.diff.u1.c4    as *const RW<u8>, "c4");
        }
    }
}
