use volatile_register::{RO, RW};

const PIT_ADDR: usize = 0x4003_7000;

#[repr(C, packed)]
struct PitRegs {
    mcr: RW<u32>,

    _pad0: [u8; 220],

    ltmr64h: RO<u32>,
    ltmr64l: RO<u32>,

    _pad1: [u8; 24],

    timer: [TimerRegs; 2],
}

#[repr(C, packed)]
struct TimerRegs {
    ldvaln: RW<u32>,
    cvaln:  RO<u32>,
    tctrln: RW<u32>,
    tflgn:  RW<u32>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn pit_test() {
        unsafe {
            let reg = & *(PIT_ADDR as *const PitRegs);
            assert_eq!(0x4003_7000 as *const RW<u32>, &reg.mcr                  as *const RW<u32>, "mcr");
            assert_eq!(0x4003_70E0 as *const RO<u32>, &reg.ltmr64h              as *const RO<u32>, "ltmr64h");
            assert_eq!(0x4003_70E4 as *const RO<u32>, &reg.ltmr64l              as *const RO<u32>, "ltmr64l");
            assert_eq!(0x4003_7100 as *const RW<u32>, &reg.timer[0].ldvalx    as *const RW<u32>, "ldval0");
            assert_eq!(0x4003_7104 as *const RO<u32>, &reg.timer[0].cvalx     as *const RO<u32>, "cval0");
            assert_eq!(0x4003_7108 as *const RW<u32>, &reg.timer[0].tctrlx    as *const RW<u32>, "tctrl0");
            assert_eq!(0x4003_710C as *const RW<u32>, &reg.timer[0].tflgx     as *const RW<u32>, "tflg0");
            assert_eq!(0x4003_7110 as *const RW<u32>, &reg.timer[1].ldvalx    as *const RW<u32>, "ldval1");
            assert_eq!(0x4003_7114 as *const RO<u32>, &reg.timer[1].cvalx     as *const RO<u32>, "cval1");
            assert_eq!(0x4003_7118 as *const RW<u32>, &reg.timer[1].tctrlx    as *const RW<u32>, "tctrl1");
            assert_eq!(0x4003_711C as *const RW<u32>, &reg.timer[1].tflgx     as *const RW<u32>, "tflg1");
        }
    }
}
