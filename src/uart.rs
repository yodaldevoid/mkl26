use core::fmt;
use core::fmt::Write;

use read;

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
    ma1:    RW<u8>,
    ma2:    RW<u8>,
    c4:     RW<u8>,
    c5:     RW<u8>,
    ed:     RO<u8>,
    modem:  RW<u8>,
    ir:     RW<u8>,

    _pad0: u8,

    pfifo:  RW<u8>,
    cfifo:  RW<u8>,
    sfifo:  RW<u8>,
    twfifo: RW<u8>,
    tcfifo: RO<u8>,
    rwfifo: RW<u8>,
    rcfifo: RO<u8>,

    _pad1: u8,

    c7816:  RW<u8>,
    ie7816: RW<u8>,
    is7816: RW<u8>,
    wp7816tx: RW<u8>,
    wn7816: RW<u8>,
    wf7816: RW<u8>,
    et7816: RW<u8>,
    tl7816: RW<u8>,
/*
    _pad2: u8,

    c6:     RW<u8>,
    pcth:   RW<u8>,
    pctl:   RW<u8>,
    b1t:    RW<u8>,
    sdth:   RW<u8>,
    sdtl:   RW<u8>,
    pre:    RW<u8>,
    tpl:    RW<u8>,
    ie:     RW<u8>,
    wb:     RW<u8>,
    s3:     RW<u8>,
    s4:     RW<u8>,
    rpl:    RO<u8>,
    rprel:  RO<u8>,
    cpw:    RW<u8>,
    ridt:   RW<u8>,
    tidt:   RW<u8>,
*/
}

pub struct Uart<'a, 'b> {
    reg: &'static mut UartRegs,
    _rx: Option<UartRx<'a>>,
    _tx: Option<UartTx<'b>>,
    _gate: ClockGate
}

// TODO: write a drop impl
// TODO: flow control
impl<'a, 'b> Uart<'a, 'b> {
    pub unsafe fn new(bus: u8,
                      rx: Option<UartRx<'a>>,
                      tx: Option<UartTx<'b>>,
                      clkdiv: (u16,u8),
                      rxfifo: bool,
                      txfifo: bool,
                      gate: ClockGate)
                      -> Result<Uart<'a, 'b>, ()> {
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
        if clkdiv.0 >= 8192 {
            return Err(())
        }
        if clkdiv.1 >= 32 {
            return Err(())
        }

        let reg = match bus {
            0 => &mut *(0x4006A000 as *mut UartRegs),
            1 => &mut *(0x4006B000 as *mut UartRegs),
            2 => &mut *(0x4006C000 as *mut UartRegs),
            _ => unimplemented!()
        };

        reg.c4.modify(|mut c4| {
            c4.set_bits(0..5, clkdiv.1);
            c4
        });
        reg.bdh.modify(|mut bdh| {
            bdh.set_bits(0..5, clkdiv.0.get_bits(8..13) as u8);
            bdh
        });
        reg.bdl.write(clkdiv.0.get_bits(0..8) as u8);

        let mut pfifo = 0;
        pfifo.set_bit(7, txfifo);
        pfifo.set_bit(3, rxfifo);
        reg.pfifo.write(pfifo);

        reg.c2.modify(|mut c2| {
            c2.set_bit(2, rx.is_some());
            c2.set_bit(3, tx.is_some());
            c2
        });

        Ok(Uart {reg: reg, _tx: tx, _rx: rx, _gate: gate})
    }
/*
    pub fn read_byte(&mut self) -> Result<u8, read::Error> {
        // wait for something in the rx buffer
        while !self.reg.s1.read().get_bit(5) {}
        Ok(self.reg.d.read())
    }
*/
    pub fn write_byte(&mut self, b: u8) -> Result<(), ()> {
        // wait for tx buffer to not be full
        while !self.reg.s1.read().get_bit(7) {}
        unsafe { self.reg.d.write(b); }
        // wait for tx complete
        while !self.reg.s1.read().get_bit(6) {}
        Ok(())
    }
}

impl<'a, 'b> Write for Uart<'a, 'b> {
    // TODO: make asynchronous
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            // wait for tx buffer to not be full
            while !self.reg.s1.read().get_bit(7) {}
            unsafe { self.reg.d.write(b); }
        }
        // wait for tx complete
        while !self.reg.s1.read().get_bit(6) {}
        Ok(())
    }
}

impl<'a, 'b> read::Read for Uart<'a, 'b> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, read::Error> {
        if buf.len() == 0 {
            return Ok(0);
        }

        // wait for something in the rx buffer
        // uncomment for synchronous read
        //while !self.reg.s1.read().get_bit(5) {}

        let mut index: usize = 0;
        let rxcount = self.reg.rcfifo.read() as usize;

        if rxcount == 0 {
            return Ok(0)
        }

        while index < rxcount - 1 && index < buf.len() - 1 {
            buf[index] = self.reg.d.read();
            index += 1;
        }

        // Clear the rdrf flag
        let _ = self.reg.s1.read();

        buf[index] = self.reg.d.read();

        Ok(buf.len())
    }
}
