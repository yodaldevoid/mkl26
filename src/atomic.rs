use core::cell::UnsafeCell;
use core::ops::{Add, BitAnd, BitOr, BitXor, Sub};

use cortex_m::interrupt;
use volatile_register::{RO, WO};

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
pub struct BmeAtomic<T>
where
    T: Copy,
{
    _reg: T,
}

impl<T> BmeAtomic<T>
where
    T: Copy,
{
    /// Performs a logical AND of the atomic and `val`, storing the result in the atomic.
    #[inline]
    pub unsafe fn and(&mut self, val: T) {
        const BME_AND_FLAG: u32 = 1 << 26;
        const BME_AND_ADDR_MASK: u32 = 0xE00F_FFFF;

        let tmp =
            &mut *(((self as *mut Self as u32) & BME_AND_ADDR_MASK | BME_AND_FLAG) as *mut WO<T>);
        tmp.write(val);
    }

    /// Performs a logical OR of the atomic and `val`, storing the result in the atomic.
    #[inline]
    pub unsafe fn or(&mut self, val: T) {
        const BME_OR_FLAG: u32 = 2 << 26;
        const BME_OR_ADDR_MASK: u32 = 0xE00F_FFFF;

        let tmp =
            &mut *(((self as *mut Self as u32) & BME_OR_ADDR_MASK | BME_OR_FLAG) as *mut WO<T>);
        tmp.write(val);
    }

    /// Performs a logical XOR of the atomic and `val`, storing the result in the atomic.
    #[inline]
    pub unsafe fn xor(&mut self, val: T) {
        const BME_XOR_FLAG: u32 = 3 << 26;
        const BME_XOR_ADDR_MASK: u32 = 0xE00F_FFFF;

        let tmp =
            &mut *(((self as *mut Self as u32) & BME_XOR_ADDR_MASK | BME_XOR_FLAG) as *mut WO<T>);
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

        let tmp = &mut *(((self as *mut Self as u32) & BME_BFI_ADDR_MASK
            | bme_bfi_flags(offset, width)) as *mut WO<T>);
        tmp.write(val);
    }

    /// Loads a bit from memory and then clears it in memory.
    #[inline]
    pub unsafe fn lac1(&mut self, offset: usize) -> T {
        const BME_LAC1_ADDR_MASK: u32 = 0xE00F_FFFF;

        /// bit must be less than 32
        const fn bme_lac1_flags(bit: usize) -> u32 {
            ((2 << 26) | (bit << 21)) as u32
        }

        let tmp = &mut *(((self as *mut Self as u32) & BME_LAC1_ADDR_MASK | bme_lac1_flags(offset))
            as *mut RO<T>);
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

        let tmp = &mut *(((self as *mut Self as u32) & BME_LAS1_ADDR_MASK | bme_las1_flags(offset))
            as *mut RO<T>);
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

        let tmp = &*(((self as *const Self as u32) & BME_UBFX_ADDR_MASK
            | bme_ubfx_flags(offset, width)) as *const RO<T>);
        tmp.read()
    }

    #[inline]
    pub fn read(&self) -> T {
        let tmp: &RO<T> = unsafe { &*(self as *const Self as *const RO<T>) };
        tmp.read()
    }
}

/// A type which can be safely shared between threads.
///
/// This "atomic" fakes atomicity by disabling interrupts for the duration of the operation.
///
/// This type has the same in-memory representation as the underlying type. For more about the
/// differences between atomic types and non-atomic types, please see [core::sync::atomic].
///
/// [core::sync::atomic]: https://doc.rust-lang.org/std/sync/atomic/index.html
// TODO: Debug, maybe
pub struct InterruptAtomic<T> {
    inner: UnsafeCell<T>,
}

// TODO: somehow support core::sync::Ordering
impl<T> InterruptAtomic<T> {
    /// Creates a new atomic.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let atomic_forty_two  = InterruptAtomic::<u32>::new(42);
    /// ```
    #[inline]
    pub const fn new(val: T) -> InterruptAtomic<T> {
        InterruptAtomic {
            inner: UnsafeCell::new(val),
        }
    }

    /// Returns a mutable reference to the underlying data.
    ///
    /// This is safe because the mutable reference guarantees that no other threads are concurrently
    /// accessing the atomic data.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let mut some_isize = InterruptAtomic::<isize>::new(10);
    /// assert_eq!(*some_isize.get_mut(), 10);
    /// *some_isize.get_mut() = 5;
    /// assert_eq!(some_isize.load(), 5);
    /// ```
    #[inline]
    pub fn get_mut(&mut self) -> &mut T {
        unsafe { &mut *self.inner.get() }
    }

    /// Consumes the atomic and returns the contained value.
    ///
    /// This is safe because passing `self` by value guarantees that no other threads are
    /// concurrently accessing the atomic data.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let mut some_isize = InterruptAtomic::<isize>::new(10);
    /// assert_eq!(some_isize.into_inner(), 5);
    /// ```
    #[inline]
    pub fn into_inner(self) -> T {
        self.inner.into_inner()
    }

    /// Loads a value from the atomic.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let some_isize = InterruptAtomic::<isize>::new(5);
    ///
    /// assert_eq!(some_isize.load(), 5);
    /// ```
    #[inline]
    pub fn load(&self) -> T
    where
        T: Copy,
    {
        interrupt::free(|_| unsafe { *self.inner.get() })
    }

    /// Stores a value into the atomic.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let some_isize = InterruptAtomic::<isize>::new(5);
    ///
    /// some_isize.store(10, Ordering::Relaxed);
    /// assert_eq!(some_isize.load(), 10);
    /// ```
    #[inline]
    pub fn store(&self, val: T) {
        interrupt::free(|_| unsafe {
            *self.inner.get() = val;
        })
    }

    /// Stores a value into the atomic, returning the previous value.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let some_isize = InterruptAtomic::<isize>::new(5);
    ///
    /// assert_eq!(some_isize.swap(10), 5);
    /// assert_eq!(some_isize.load(), 10);
    /// ```
    #[inline]
    pub fn swap(&self, val: T) -> T
    where
        T: Copy,
    {
        interrupt::free(|_| unsafe {
            let tmp = *self.inner.get();
            *self.inner.get() = val;
            tmp
        })
    }

    /// Stores a value into the atomic if the current value is the same as the `current` value.
    ///
    /// The return value is always the previous value. If it is equal to `current`, then the value
    /// was updated.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let some_isize = InterruptAtomic::<isize>::new(5);
    ///
    /// assert_eq!(some_isize.compare_and_swap(5, 10), 5);
    /// assert_eq!(some_isize.load(), 10);
    ///
    /// assert_eq!(some_isize.compare_and_swap(6, 12), 10);
    /// assert_eq!(some_isize.load(), 10);
    /// ```
    #[inline]
    pub fn compare_and_swap(&self, current: T, new: T) -> T
    where
        T: Copy + PartialEq,
    {
        interrupt::free(|_| unsafe {
            let tmp = *self.inner.get();
            if tmp == current {
                *self.inner.get() = new;
            }
            tmp
        })
    }

    /// Stores a value into the atomic if the current value is the same as the `current` value.
    ///
    /// The return value is a result indicating whether the new value was written and containing the
    /// previous value. On success this value is guaranteed to be equal to `current`.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let some_isize = InterruptAtomic::<isize>::new(5);
    ///
    /// assert_eq!(some_isize.compare_exchange(5, 10), Ok(5));
    /// assert_eq!(some_isize.load(), 10);
    ///
    /// assert_eq!(some_isize.compare_exchange(6, 12), Err(10));
    /// assert_eq!(some_isize.load(), 10);
    /// ```
    #[inline]
    pub fn compare_and_exchange(&self, current: T, new: T) -> Result<T, T>
    where
        T: Copy + PartialEq,
    {
        interrupt::free(|_| unsafe {
            let tmp = *self.inner.get();
            if tmp == current {
                *self.inner.get() = new;
                Ok(tmp)
            } else {
                Err(tmp)
            }
        })
    }

    /*
    /// Stores a value into the atomic if the current value is the same as the `current` value.
    ///
    /// Unlike [`compare_exchange`], this function is allowed to spuriously fail even when the
    /// comparison succeeds, which can result in more efficient code on some platforms. The return
    /// value is a result indicating whether the new value was written and containing the previous
    /// value.
    ///
    /// [`compare_exchange`]: #method.compare_exchange
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let val = InterruptAtomic::<isize>::new(4);
    ///
    /// let mut old = val.load();
    /// loop {
    ///     let new = old * 2;
    ///     match val.compare_exchange_weak(old, new) {
    ///         Ok(_) => break,
    ///         Err(x) => old = x,
    ///     }
    /// }
    /// ```
    #[inline]
    pub fn compare_exchange_weak(
        &self,
        current: T,
        new: T
    ) -> Result<T, T> where T: Copy + PartialEq {
        interrupt::free(|_| {
            unsafe {
                let tmp = *self.inner.get();
                if tmp == current {
                    *self.inner.get() = new;
                    Ok(tmp)
                } else {
                    Err(tmp)
                }
            }
        })
    }
    */

    /// Adds to the current value, returning the previous value.
    ///
    /// This operation is *not* guaranteed to wrap around on overflow.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let foo = InterruptAtomic::<isize>::new(0);
    /// assert_eq!(foo.fetch_add(10), 0);
    /// assert_eq!(foo.load(), 10);
    /// ```
    #[inline]
    pub fn fetch_add(&self, val: T) -> T
    where
        T: Copy + Add<Output = T>,
    {
        interrupt::free(|_| unsafe {
            let tmp = *self.inner.get();
            *self.inner.get() = tmp + val;
            tmp
        })
    }

    /// Subtracts from the current value, returning the previous value.
    ///
    /// This operation is *not* guaranteed to wrap around on overflow.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let foo = InterruptAtomic::<isize>::new(0);
    /// assert_eq!(foo.fetch_add(10), 0);
    /// assert_eq!(foo.load(), -10);
    /// ```
    #[inline]
    pub fn fetch_sub(&self, val: T) -> T
    where
        T: Copy + Sub<Output = T>,
    {
        interrupt::free(|_| unsafe {
            let tmp = *self.inner.get();
            *self.inner.get() = tmp - val;
            tmp
        })
    }

    /// Bitwise "and" with the current value.
    ///
    /// Performs a bitwise "and" operation on the current value and the argument `val`, and sets the
    /// new value to the result.
    ///
    /// Returns the previous value.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let foo = InterruptAtomic::<isize>::new(0b101101);
    /// assert_eq!(foo.fetch_and(0b110011), 0b101101);
    /// assert_eq!(foo.load(), 0b100001);
    /// ```
    #[inline]
    pub fn fetch_and(&self, val: T) -> T
    where
        T: Copy + BitAnd<Output = T>,
    {
        interrupt::free(|_| unsafe {
            let tmp = *self.inner.get();
            *self.inner.get() = tmp & val;
            tmp
        })
    }

    /// Bitwise "or" with the current value.
    ///
    /// Performs a bitwise "or" operation on the current value and the argument `val`, and sets the
    /// new value to the result.
    ///
    /// Returns the previous value.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let foo = InterruptAtomic::<isize>::new(0b101101);
    /// assert_eq!(foo.fetch_or(0b110011), 0b101101);
    /// assert_eq!(foo.load(), 0b111111);
    /// ```
    #[inline]
    pub fn fetch_or(&self, val: T) -> T
    where
        T: Copy + BitOr<Output = T>,
    {
        interrupt::free(|_| unsafe {
            let tmp = *self.inner.get();
            *self.inner.get() = tmp | val;
            tmp
        })
    }

    /// Bitwise "xor" with the current value.
    ///
    /// Performs a bitwise "xor" operation on the current value and the argument `val`, and sets the
    /// new value to the result.
    ///
    /// Returns the previous value.
    ///
    /// # Examples
    ///
    /// ```
    /// use mkl26::atomic::InterruptAtomic;
    ///
    /// let foo = InterruptAtomic::<isize>::new(0b101101);
    /// assert_eq!(foo.fetch_xor(0b110011), 0b101101);
    /// assert_eq!(foo.load(), 0b011110);
    /// ```
    #[inline]
    pub fn fetch_xor(&self, val: T) -> T
    where
        T: Copy + BitXor<Output = T>,
    {
        interrupt::free(|_| unsafe {
            let tmp = *self.inner.get();
            *self.inner.get() = tmp ^ val;
            tmp
        })
    }
}

unsafe impl<T> Sync for InterruptAtomic<T> where T: Sync {}

impl<T> Default for InterruptAtomic<T>
where
    T: Default,
{
    fn default() -> Self {
        Self::new(T::default())
    }
}

impl<T> From<T> for InterruptAtomic<T> {
    fn from(val: T) -> InterruptAtomic<T> {
        Self::new(val)
    }
}
