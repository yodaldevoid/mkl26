//! Exceptions

/// Fault and system exceptions
#[allow(non_camel_case_types)]
#[doc(hidden)]
pub enum Exception {
    /// Non-maskable interrupt
    NMI,
    /// All class of fault.
    HARD_FAULT,
    /// Memory management.
    MEN_MANAGE,
    /// Pre-fetch fault, memory access fault.
    BUS_FAULT,
    /// Undefined instruction or illegal state.
    USAGE_FAULT,
    /// System service call via SWI instruction
    SVCALL,
    /// Pendable request for system service
    PENDSV,
    /// System tick timer
    SYS_TICK,
    DEBUG,
}

#[export_name = "DEFAULT_HANDLER"]
#[linkage = "weak"]
pub(crate) extern "C" fn default_handler() -> ! {
    //asm::bkpt();

    loop {}
}

// make sure the compiler emits the DEFAULT_HANDLER symbol so the linker can
// find it!
#[used]
static KEEP: extern "C" fn() -> ! = default_handler;

/// This macro lets you override the default exception handler
///
/// The first and only argument to this macro is the path to the function that
/// will be used as the default handler. That function must have signature
/// `fn()`
///
/// # Examples
///
/// ``` ignore
/// default_handler!(foo::bar);
///
/// mod foo {
///     pub fn bar() {
///         ::cortex_m::asm::bkpt();
///         loop {}
///     }
/// }
/// ```
#[macro_export]
macro_rules! default_handler {
    ($path:path) => {
        #[allow(non_snake_case)]
        #[doc(hidden)]
        #[no_mangle]
        pub unsafe extern "C" fn DEFAULT_HANDLER() {
            // type checking
            let f: fn() = $path;
            f();
        }
    }
}

/// Assigns a handler to an exception
///
/// This macro takes two arguments: the name of an exception and the path to the
/// function that will be used as the handler of that exception. That function
/// must have signature `fn()`.
///
/// Optionally, a third argument may be used to declare exception local data.
/// The handler will have exclusive access to these *local* variables on each
/// invocation. If the third argument is used then the signature of the handler
/// function must be `fn(&mut $NAME::Locals)` where `$NAME` is the first
/// argument passed to the macro.
///
/// # Example
///
/// ``` ignore
/// exception!(MEM_MANAGE, mpu_fault);
///
/// fn mpu_fault() {
///     panic!("Oh no! Something went wrong");
/// }
///
/// exception!(SYS_TICK, periodic, locals: {
///     counter: u32 = 0;
/// });
///
/// fn periodic(locals: &mut SYS_TICK::Locals) {
///     locals.counter += 1;
///     println!("This function has been called {} times", locals.counter);
/// }
/// ```
#[macro_export]
macro_rules! exception {
    ($NAME:ident, $path:path, locals: {
        $($lvar:ident:$lty:ident = $lval:expr;)+
    }) => {
        #[allow(non_snake_case)]
        mod $NAME {
            pub struct Locals {
                $(
                    pub $lvar: $lty,
                )+
            }
        }

        #[allow(non_snake_case)]
        #[doc(hidden)]
        #[no_mangle]
        pub unsafe extern "C" fn $NAME() {
            // check that the handler exists
            let _ = $crate::Exception::$NAME;

            static mut LOCALS: self::$NAME::Locals = self::$NAME::Locals {
                $(
                    $lvar: $lval,
                )*
            };

            // type checking
            let f: fn(&mut self::$NAME::Locals) = $path;
            f(&mut LOCALS);
        }
    };
    ($NAME:ident, $path:path) => {
        #[allow(non_snake_case)]
        #[doc(hidden)]
        #[no_mangle]
        pub unsafe extern "C" fn $NAME() {
            // check that the handler exists
            let _ = $crate::Exception::$NAME;

            // type checking
            let f: fn() = $path;
            f();
        }
    }
}

#[link_section = ".vectors.exceptions"]
#[no_mangle]
pub static EXCEPTIONS: [Option<unsafe extern "C" fn()>; 14] = [
    Some(NMI),
    Some(HARD_FAULT),
    Some(MEM_MANAGE),
    Some(BUS_FAULT),
    Some(USAGE_FAULT),
    None,
    None,
    None,
    None,
    Some(SVCALL),
    Some(DEBUG),
    None,
    Some(PENDSV),
    Some(SYS_TICK),
];

#[linkage = "weak"]
#[no_mangle]
extern "C" fn NMI() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern "C" fn HARD_FAULT() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern "C" fn MEM_MANAGE() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern "C" fn BUS_FAULT() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern "C" fn USAGE_FAULT() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern "C" fn SVCALL() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern "C" fn DEBUG() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern "C" fn PENDSV() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern "C" fn SYS_TICK() { default_handler() }
