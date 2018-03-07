use core::fmt::{self, Write};
use core::marker::PhantomData;

use volatile_register::{RO,RW};
use bit_field::BitField;

use port::{UartRx, UartTx};
use sim::ClockGate;

#[repr(C,packed)]
struct UartRegs {
    bdh:    RW<u8>,
    bdl:    RW<u8>,
    c1:     RW<u8>,
    c2:     RW<u8>,
    s1:     RO<u8>,
    s2:     RW<u8>,
    c3:     RW<u8>,
    d:      RW<u8>,
    diff:   UartDiff,
}

#[repr(C)]
union UartDiff {
    u0: Uart0Diff,
    u1: Uart12Diff,
}

#[repr(C,packed)]
struct Uart0Diff {
    ma1:    RW<u8>,
    ma2:    RW<u8>,
    c4:     RW<u8>,
    c5:     RW<u8>,
}

#[repr(C,packed)]
struct Uart12Diff {
    c4:     RW<u8>,
}

/// A UART peripheral
///
/// T is the "character" size used for the UART. This can be 8, 9, or 10 bits.
// TODO: consider grabbing crate ux
pub struct Uart<'a, 'b, B> {
    reg: &'static mut UartRegs,
    _rx: Option<UartRx<'a>>,
    _tx: Option<UartTx<'b>>,
    _gate: ClockGate,
    _bus: u8, // TODO: replace with a const generic when that comes around
    _char: PhantomData<B>,
}

/// clock_freq - Frequency of module clock in Hz. UART0 uses the clock specified by
/// SIM_SOPT2[UART0SRC]. UART1 and UART2 use the bus clock.
pub fn calc_clkdiv(baud: u32, clock_freq: u32) -> u16 {
    ((((clock_freq / 16)  + (baud / 2)) / baud) & 0xFFF) as u16
}

// TODO: flow control
// TODO: support ISR mode
// TODO: support DMA mode
// TODO: 9 bit mode
// TODO: 10 bit mode
// TODO: stop bits
impl<'a, 'b> Uart<'a, 'b, u8> {
    pub unsafe fn new(bus: u8,
                      rx: Option<UartRx<'a>>,
                      tx: Option<UartTx<'b>>,
                      clkdiv: u16,
                      gate: ClockGate)
                      -> Result<Uart<'a, 'b, u8>, ()> {
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
            return Err(())
        }

        let reg = match bus {
            0 => &mut *(0x4006A000 as *mut UartRegs),
            1 => &mut *(0x4006B000 as *mut UartRegs),
            2 => &mut *(0x4006C000 as *mut UartRegs),
            _ => unreachable!()
        };

        reg.bdh.modify(|mut bdh| {
            bdh.set_bits(0..4, clkdiv.get_bits(8..12) as u8);
            bdh
        });
        reg.bdl.write(clkdiv.get_bits(0..8) as u8);

        reg.c2.modify(|mut c2| {
            c2.set_bit(2, rx.is_some());
            c2.set_bit(3, tx.is_some());
            c2
        });

        Ok(Uart { reg: reg, _tx: tx, _rx: rx, _gate: gate, _bus: bus, _char: PhantomData })
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
            unsafe { self.reg.d.write(b); }
            Ok(())
        } else {
            Err(())
        }
    }

    // TODO: add timeout
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        if buf.len() == 0 {
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
        if buf.len() == 0 {
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
    // TODO: add timeout
    pub fn flush(&self) -> Result<(), ()> {
        while !self.reg.s1.read().get_bit(6) {}
        Ok(())
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
