use volatile_register::RW;
use bit_field::BitField;

#[repr(C,packed)]
pub struct Watchdog {
    stctrlh:    RW<u16>,
    stctrll:    RW<u16>,
    tovalh:     RW<u16>,
    tovall:     RW<u16>,
    winh:       RW<u16>,
    winl:       RW<u16>,
    refresh:    RW<u16>,
    unlock:     RW<u16>,
    tmrouth:    RW<u16>,
    tmroutl:    RW<u16>,
    rstcnt:     RW<u16>,
    presc:      RW<u16>
}

impl Watchdog {
    pub unsafe fn new() -> &'static mut Watchdog {
        &mut *(0x40052000 as *mut Watchdog)
    }

    pub fn disable(&mut self) {
        unsafe{
            self.unlock.write(0xC520);
            self.unlock.write(0xD928);
            asm!("nop" : : : "memory");
            asm!("nop" : : : "memory");
            self.stctrlh.modify(|mut ctrl| {
                ctrl.set_bit(0, false);
                ctrl
            });
        }
    }
}
