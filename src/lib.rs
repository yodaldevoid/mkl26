#![feature(compiler_builtins_lib)]
#![feature(core_intrinsics)]
#![feature(lang_items)]
#![feature(naked_functions)]
#![no_std]

extern crate bit_field;
extern crate volatile_register;
extern crate cortex_m;
extern crate cortex_m_rt;

pub mod adc;
pub mod mcg;
pub mod osc;
pub mod port;
pub mod read;
pub mod sim;
pub mod uart;
pub mod watchdog;
pub mod interrupts;
