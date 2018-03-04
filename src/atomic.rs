use volatile_register::{RO,WO};

/// Generates atomic operations using the Bit Manipulation Engine (BME).
///
/// T may only be 8, 16, or 32 bits wide.
///
/// Functions that write (all functions other than `read` and `ubfx`) are `unsafe` due to possible
/// side effects. Admittedly, reading can also have side effects.
///
/// All operations are volatile due to address manipulations required to use the BME causing writes
/// and reads to appear to be unnecessary.
///
/// The BME only makes operations targeted at the peripheral address space atomic. Operations are
/// two cycle atomic (inseparable) operations.
pub struct BmeAtomic<T> where T: Copy {
    _reg: T
}

impl<T> BmeAtomic<T> where T: Copy {
    /// Performs a logical AND of the atomic and `val`, storing the result in the atomic.
    #[inline]
    pub unsafe fn and(&mut self, val: T) {
        const BME_AND_FLAG: u32 = 1 << 26;
        const BME_AND_ADDR_MASK: u32 = 0xE00FFFFF;

        let tmp = &mut *(((self as *mut Self as u32) & BME_AND_ADDR_MASK | BME_AND_FLAG) as *mut WO<T>);
        tmp.write(val);
    }

    /// Performs a logical OR of the atomic and `val`, storing the result in the atomic.
    #[inline]
    pub unsafe fn or(&mut self, val: T) {
        const BME_OR_FLAG: u32 = 2 << 26;
        const BME_OR_ADDR_MASK: u32 = 0xE00F_FFFF;

        let tmp = &mut *(((self as *mut Self as u32) & BME_OR_ADDR_MASK | BME_OR_FLAG) as *mut WO<T>);
        tmp.write(val);
    }

    /// Performs a logical XOR of the atomic and `val`, storing the result in the atomic.
    #[inline]
    pub unsafe fn xor(&mut self, val: T) {
        const BME_XOR_FLAG: u32 = 3 << 26;
        const BME_XOR_ADDR_MASK: u32 = 0xE00F_FFFF;

        let tmp = &mut *(((self as *mut Self as u32) & BME_XOR_ADDR_MASK | BME_XOR_FLAG) as *mut WO<T>);
        tmp.write(val);
    }

    /// Bit field insert
    ///
    /// Does not work with the FGPIO peripheral at 0x400F_F000. Still works with
    /// the GPIO peripheral at 0x4000_F000.
    #[inline]
    pub unsafe fn bfi(&mut self, val: T, offset: usize, width: usize) {
        const BME_BFI_ADDR_MASK: u32 = 0xE007_FFFF;

        /// bit must be less than 32
        /// width must be less than 16
        const fn bme_bfi_flags(bit: usize, width: usize) -> u32 {
            ((1 << 28) | (bit << 23) | ((width - 1) << 19)) as u32
        }

        let tmp = &mut *(((self as *mut Self as u32) & BME_BFI_ADDR_MASK | bme_bfi_flags(offset, width)) as *mut WO<T>);
        tmp.write(val);
    }

    /// Loads a bit from memory and then clears it in memory.
    #[inline]
    pub unsafe fn lac1(&mut self, offset: usize) -> T {
        const BME_LAC1_ADDR_MASK: u32 = 0x0E00F_FFFF;

        /// bit must be less than 32
        const fn bme_lac1_flags(bit: usize) -> u32 {
            ((2 << 26) | (bit << 21)) as u32
        }

        let tmp = &mut *(((self as *mut Self as u32) & BME_LAC1_ADDR_MASK | bme_lac1_flags(offset)) as *mut RO<T>);
        tmp.read()
    }

    /// Loads a bit from memory and then set it in memory.
    #[inline]
    pub unsafe fn las1(&mut self, offset: usize) -> T {
        const BME_LAS1_ADDR_MASK: u32 = 0xE00F_FFFF;

        /// bit must be less than 32
        const fn bme_las1_flags(bit: usize) -> u32 {
            ((3 << 26) | (bit << 21)) as u32
        }

        let tmp = &mut *(((self as *mut Self as u32) & BME_LAS1_ADDR_MASK | bme_las1_flags(offset)) as *mut RO<T>);
        tmp.read()
    }

    /// Unsigned bit field extract
    ///
    /// Does not work with the FGPIO peripheral at 0x400F_F000. Still works with
    /// the GPIO peripheral at 0x4000_F000.
    #[inline]
    pub unsafe fn ubfx(&self, offset: usize, width: usize) -> T {
        const BME_UBFX_ADDR_MASK: u32 = 0xE007_FFFF;

        /// bit must be less than 32
        /// width must be less than 16
        const fn bme_ubfx_flags(bit: usize, width: usize) -> u32 {
            ((1 << 28) | (bit << 23) | ((width - 1) << 19)) as u32
        }

        let tmp = &*(((self as *const Self as u32) & BME_UBFX_ADDR_MASK | bme_ubfx_flags(offset, width)) as *const RO<T>);
        tmp.read()
    }

    #[inline]
    pub fn read(&self) -> T {
        let tmp: &RO<T> = unsafe { &*(self as *const Self as *const RO<T>) };
        tmp.read()
    }
}
