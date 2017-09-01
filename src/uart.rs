use core::fmt;

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
}

pub struct Uart<'a, 'b> {
    reg: &'static mut UartRegs,
    _rx: Option<UartRx<'a>>,
    _tx: Option<UartTx<'b>>,
    _gate: ClockGate
}

impl<'a, 'b> Uart<'a, 'b> {
    pub unsafe fn new(id: u8,
                      rx: Option<UartRx<'a>>,
                      tx: Option<UartTx<'b>>,
                      clkdiv: (u16,u8),
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

        reg.c2.update(|c2| {
            c2.set_bit(2, rx.is_some());
            c2.set_bit(3, tx.is_some());
        });

        Ok(Uart {reg: reg, _tx: tx, _rx: rx, _gate: gate})
    }
}

impl<'a, 'b> fmt::Write for Uart<'a, 'b> {
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
