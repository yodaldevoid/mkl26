#![feature(compiler_builtins_lib)]
#![feature(const_fn)]
#![feature(global_asm)]
#![feature(untagged_unions)]
#![no_std]

extern crate arraydeque;
extern crate bare_metal;
extern crate bit_field;
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate ux;
extern crate volatile_register;

#[macro_use]
pub mod interrupts;

pub mod adc;
pub mod atomic;
pub mod i2c;
pub mod mcg;
pub mod osc;
pub mod port;
pub mod sim;
pub mod spi;
pub mod uart;
