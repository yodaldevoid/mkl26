pub struct Error;

/// The `Read` trait allows for reading bytes from a source.
///
/// Implementors of the `Read` trait are called 'readers'.
///
/// Readers are defined by one required method, [`read()`]. Each call to [`read()`]
/// will attempt to pull bytes from this source into a provided buffer. A
/// number of other methods are implemented in terms of [`read()`], giving
/// implementors a number of ways to read bytes while only needing to implement
/// a single method.
///
/// Readers are intended to be composable with one another. Many implementors
/// throughout [`std::io`] take and provide types which implement the `Read`
/// trait.
///
/// Please note that each call to [`read()`] may involve a system call, and
/// therefore, using something that implements [`BufRead`], such as
/// [`BufReader`], will be more efficient.
///
/// # Examples
///
/// [`File`]s implement `Read`:
///
/// [`read()`]: trait.Read.html#tymethod.read
/// [`std::io`]: ../../std/io/
/// [`File`]: ../fs/struct.File.html
/// [`BufRead`]: trait.BufRead.html
/// [`BufReader`]: struct.BufReader.html
///
/// ```
/// use std::io;
/// use std::io::prelude::*;
/// use std::fs::File;
///
/// # fn foo() -> io::Result<()> {
/// let mut f = File::open("foo.txt")?;
/// let mut buffer = [0; 10];
///
/// // read up to 10 bytes
/// f.read(&mut buffer)?;
///
/// let mut buffer = vec![0; 10];
/// // read the whole file
/// f.read_to_end(&mut buffer)?;
///
/// // read into a String, so that you don't need to do the conversion.
/// let mut buffer = String::new();
/// f.read_to_string(&mut buffer)?;
///
/// // and more! See the other methods for more details.
/// # Ok(())
/// # }
/// ```
pub trait Read {
    /// Pull some bytes from this source into the specified buffer, returning
    /// how many bytes were read.
    ///
    /// This function does not provide any guarantees about whether it blocks
    /// waiting for data, but if an object needs to block for a read but cannot
    /// it will typically signal this via an [`Err`] return value.
    ///
    /// If the return value of this method is [`Ok(n)`], then it must be
    /// guaranteed that `0 <= n <= buf.len()`. A nonzero `n` value indicates
    /// that the buffer `buf` has been filled in with `n` bytes of data from this
    /// source. If `n` is `0`, then it can indicate one of two scenarios:
    ///
    /// 1. This reader has reached its "end of file" and will likely no longer
    ///    be able to produce bytes. Note that this does not mean that the
    ///    reader will *always* no longer be able to produce bytes.
    /// 2. The buffer specified was 0 bytes in length.
    ///
    /// No guarantees are provided about the contents of `buf` when this
    /// function is called, implementations cannot rely on any property of the
    /// contents of `buf` being true. It is recommended that implementations
    /// only write data to `buf` instead of reading its contents.
    ///
    /// # Errors
    ///
    /// If this function encounters any form of I/O or other error, an error
    /// variant will be returned. If an error is returned then it must be
    /// guaranteed that no bytes were read.
    ///
    /// An error of the [`ErrorKind::Interrupted`] kind is non-fatal and the read
    /// operation should be retried if there is nothing else to do.
    ///
    /// # Examples
    ///
    /// [`File`]s implement `Read`:
    ///
    /// [`Err`]: ../../std/result/enum.Result.html#variant.Err
    /// [`Ok(n)`]: ../../std/result/enum.Result.html#variant.Ok
    /// [`ErrorKind::Interrupted`]: ../../std/io/enum.ErrorKind.html#variant.Interrupted
    /// [`File`]: ../fs/struct.File.html
    ///
    /// ```
    /// use std::io;
    /// use std::io::prelude::*;
    /// use std::fs::File;
    ///
    /// # fn foo() -> io::Result<()> {
    /// let mut f = File::open("foo.txt")?;
    /// let mut buffer = [0; 10];
    ///
    /// // read up to 10 bytes
    /// f.read(&mut buffer[..])?;
    /// # Ok(())
    /// # }
    /// ```
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error>;
/*
    /// Read all bytes until EOF in this source, placing them into `buf`.
    ///
    /// All bytes read from this source will be appended to the specified buffer
    /// `buf`. This function will continuously call [`read()`] to append more data to
    /// `buf` until [`read()`] returns either [`Ok(0)`] or an error of
    /// non-[`ErrorKind::Interrupted`] kind.
    ///
    /// If successful, this function will return the total number of bytes read.
    ///
    /// # Errors
    ///
    /// If this function encounters an error of the kind
    /// [`ErrorKind::Interrupted`] then the error is ignored and the operation
    /// will continue.
    ///
    /// If any other read error is encountered then this function immediately
    /// returns. Any bytes which have already been read will be appended to
    /// `buf`.
    ///
    /// # Examples
    ///
    /// [`File`]s implement `Read`:
    ///
    /// [`read()`]: trait.Read.html#tymethod.read
    /// [`Ok(0)`]: ../../std/result/enum.Result.html#variant.Ok
    /// [`ErrorKind::Interrupted`]: ../../std/io/enum.ErrorKind.html#variant.Interrupted
    /// [`File`]: ../fs/struct.File.html
    ///
    /// ```
    /// use std::io;
    /// use std::io::prelude::*;
    /// use std::fs::File;
    ///
    /// # fn foo() -> io::Result<()> {
    /// let mut f = File::open("foo.txt")?;
    /// let mut buffer = Vec::new();
    ///
    /// // read the whole file
    /// f.read_to_end(&mut buffer)?;
    /// # Ok(())
    /// # }
    /// ```
    fn read_to_end(&mut self, buf: &mut Vec<u8>) -> Result<usize, Error> {

    }
*/
/*
    /// Read all bytes until EOF in this source, placing them into `buf`.
    ///
    /// If successful, this function returns the number of bytes which were read
    /// and appended to `buf`.
    ///
    /// # Errors
    ///
    /// If the data in this stream is *not* valid UTF-8 then an error is
    /// returned and `buf` is unchanged.
    ///
    /// See [`read_to_end`][readtoend] for other error semantics.
    ///
    /// [readtoend]: #method.read_to_end
    ///
    /// # Examples
    ///
    /// [`File`][file]s implement `Read`:
    ///
    /// [file]: ../fs/struct.File.html
    ///
    /// ```
    /// use std::io;
    /// use std::io::prelude::*;
    /// use std::fs::File;
    ///
    /// # fn foo() -> io::Result<()> {
    /// let mut f = File::open("foo.txt")?;
    /// let mut buffer = String::new();
    ///
    /// f.read_to_string(&mut buffer)?;
    /// # Ok(())
    /// # }
    /// ```
    fn read_to_string(&mut self, buf: &mut String) -> Result<usize, Error> {
        // Note that we do *not* call `.read_to_end()` here. We are passing
        // `&mut Vec<u8>` (the raw contents of `buf`) into the `read_to_end`
        // method to fill it up. An arbitrary implementation could overwrite the
        // entire contents of the vector, not just append to it (which is what
        // we are expecting).
        //
        // To prevent extraneously checking the UTF-8-ness of the entire buffer
        // we pass it to our hardcoded `read_to_end` implementation which we
        // know is guaranteed to only read data into the end of the buffer.
        append_to_string(buf, |b| read_to_end(self, b))
    }
*/
/*
    /// Read the exact number of bytes required to fill `buf`.
    ///
    /// This function reads as many bytes as necessary to completely fill the
    /// specified buffer `buf`.
    ///
    /// No guarantees are provided about the contents of `buf` when this
    /// function is called, implementations cannot rely on any property of the
    /// contents of `buf` being true. It is recommended that implementations
    /// only write data to `buf` instead of reading its contents.
    ///
    /// # Errors
    ///
    /// If this function encounters an error of the kind
    /// [`ErrorKind::Interrupted`] then the error is ignored and the operation
    /// will continue.
    ///
    /// If this function encounters an "end of file" before completely filling
    /// the buffer, it returns an error of the kind [`ErrorKind::UnexpectedEof`].
    /// The contents of `buf` are unspecified in this case.
    ///
    /// If any other read error is encountered then this function immediately
    /// returns. The contents of `buf` are unspecified in this case.
    ///
    /// If this function returns an error, it is unspecified how many bytes it
    /// has read, but it will never read more than would be necessary to
    /// completely fill the buffer.
    ///
    /// # Examples
    ///
    /// [`File`]s implement `Read`:
    ///
    /// [`File`]: ../fs/struct.File.html
    /// [`ErrorKind::Interrupted`]: ../../std/io/enum.ErrorKind.html#variant.Interrupted
    /// [`ErrorKind::UnexpectedEof`]: ../../std/io/enum.ErrorKind.html#variant.UnexpectedEof
    ///
    /// ```
    /// use std::io;
    /// use std::io::prelude::*;
    /// use std::fs::File;
    ///
    /// # fn foo() -> io::Result<()> {
    /// let mut f = File::open("foo.txt")?;
    /// let mut buffer = [0; 10];
    ///
    /// // read exactly 10 bytes
    /// f.read_exact(&mut buffer)?;
    /// # Ok(())
    /// # }
    /// ```
    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        while !buf.is_empty() {
            match self.read(buf) {
                Ok(0) => break,
                Ok(n) => { let tmp = buf; buf = &mut tmp[n..]; }
                Err(ref e) if e.kind() == ErrorKind::Interrupted => {}
                Err(e) => return Err(e),
            }
        }
        if !buf.is_empty() {
            Err(Error::new(ErrorKind::UnexpectedEof,
                           "failed to fill whole buffer"))
        } else {
            Ok(())
        }
    }
*/
    /// Creates a "by reference" adaptor for this instance of `Read`.
    ///
    /// The returned adaptor also implements `Read` and will simply borrow this
    /// current reader.
    ///
    /// # Examples
    ///
    /// [`File`][file]s implement `Read`:
    ///
    /// [file]: ../fs/struct.File.html
    ///
    /// ```
    /// use std::io;
    /// use std::io::Read;
    /// use std::fs::File;
    ///
    /// # fn foo() -> io::Result<()> {
    /// let mut f = File::open("foo.txt")?;
    /// let mut buffer = Vec::new();
    /// let mut other_buffer = Vec::new();
    ///
    /// {
    ///     let reference = f.by_ref();
    ///
    ///     // read at most 5 bytes
    ///     reference.take(5).read_to_end(&mut buffer)?;
    ///
    /// } // drop our &mut reference so we can use f again
    ///
    /// // original file still usable, read the rest
    /// f.read_to_end(&mut other_buffer)?;
    /// # Ok(())
    /// # }
    /// ```
    fn by_ref(&mut self) -> &mut Self where Self: Sized { self }
/*
    fn bytes(self) -> Bytes<Self>
    where
        Self: Sized,
    {

    }

    fn chars(self) -> Chars<Self>
    where
        Self: Sized,
    {
        
    }

    fn chain<R: Read>(self, next: R) -> Chain<Self, R>
    where
        Self: Sized,
    {
        
    }

    fn take(self, limit: u64) -> Take<Self>
    where
        Self: Sized,
    {
        
    }
*/
}