use core::fmt;
use core::fmt::Write;

use read;

use volatile::Volatile;
use bit_field::BitField;

use port::{UartRx, UartTx};
use sim::ClockGate;

#[repr(C,packed)]
struct UartRegs {
    bdh: Volatile<u8>,
    bdl: Volatile<u8>,
    c1: Volatile<u8>,
    c2: Volatile<u8>,
    s1: Volatile<u8>,
    s2: Volatile<u8>,
    c3: Volatile<u8>,
    d: Volatile<u8>,
    ma1: Volatile<u8>,
    ma2: Volatile<u8>,
    c4: Volatile<u8>,
    c5: Volatile<u8>,
    ed: Volatile<u8>,
    modem: Volatile<u8>,
    ir: Volatile<u8>,

    _pad0: u8,

    pfifo: Volatile<u8>,
    cfifo: Volatile<u8>,
    sfifo: Volatile<u8>,
    twfifo: Volatile<u8>,
    tcfifo: Volatile<u8>,
    rwfifo: Volatile<u8>,
    rcfifo: Volatile<u8>,

    _pad1: u8,

    c7816: Volatile<u8>,
    ie7816: Volatile<u8>,
    is7816: Volatile<u8>,
    wp7816tx: Volatile<u8>,
    wn7816: Volatile<u8>,
    wf7816: Volatile<u8>,
    et7816: Volatile<u8>,
    tl7816: Volatile<u8>,
/*
    _pad2: u8,

    c6: Volatile<u8>,
    pcth: Volatile<u8>,
    pctl: Volatile<u8>,
    b1t: Volatile<u8>,
    sdth: Volatile<u8>,
    sdtl: Volatile<u8>,
    pre: Volatile<u8>,
    tpl: Volatile<u8>,
    ie: Volatile<u8>,
    wb: Volatile<u8>,
    s3: Volatile<u8>,
    s4: Volatile<u8>,
    rpl: Volatile<u8>,
    rprel: Volatile<u8>,
    cpw: Volatile<u8>,
    ridt: Volatile<u8>,
    tidt: Volatile<u8>,
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
    pub unsafe fn new(id: u8,
                      rx: Option<UartRx<'a>>,
                      tx: Option<UartTx<'b>>,
                      clkdiv: (u16,u8),
                      rxfifo: bool,
                      txfifo: bool,
                      gate: ClockGate)
                      -> Result<Uart<'a, 'b>, ()> {
        if let Some(r) = rx.as_ref() {
            if r.uart() != id {
                return Err(());
            }
        }
        if let Some(t) = tx.as_ref() {
            if t.uart() != id {
                return Err(());
            }
        }
        if clkdiv.0 >= 8192 {
            return Err(())
        }
        if clkdiv.1 >= 32 {
            return Err(())
        }

        let reg = match id {
            0 => &mut *(0x4006A000 as *mut UartRegs),
            1 => &mut *(0x4006B000 as *mut UartRegs),
            2 => &mut *(0x4006C000 as *mut UartRegs),
            _ => return Err(())
        };

        reg.c4.update(|c4| {
            c4.set_bits(0..5, clkdiv.1);
        });
        reg.bdh.update(|bdh| {
            bdh.set_bits(0..5, clkdiv.0.get_bits(8..13) as u8);
        });
        reg.bdl.write(clkdiv.0.get_bits(0..8) as u8);

        let mut pfifo = 0;
        pfifo.set_bit(7, txfifo);
        pfifo.set_bit(3, rxfifo);
        reg.pfifo.write(pfifo);

        reg.c2.update(|c2| {
            c2.set_bit(2, rx.is_some());
            c2.set_bit(3, tx.is_some());
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
        self.reg.d.write(b);
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
            self.reg.d.write(b);
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
