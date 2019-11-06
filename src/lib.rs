#![feature(untagged_unions)]
#![no_std]

#[macro_use]
pub mod interrupt;

pub mod adc;
pub mod atomic;
pub mod i2c;
pub mod mcg;
pub mod osc;
pub mod pit;
pub mod port;
pub mod sim;
pub mod spi;
pub mod tpm;
pub mod uart;
