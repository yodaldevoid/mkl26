//! Interrupts
use cortex_m::interrupt::Nr;

#[allow(non_camel_case_types)]
pub enum Interrupt {
    DMA_CHANNEL0,
    DMA_CHANNEL1,
    DMA_CHANNEL2,
    DMA_CHANNEL3,
    // Blank
    FLASH, // FTFA complete or collision
    LV_DETECT,
    LLWU,
    I2C0,
    I2C1,
    SPI0,
    SPI1,
    UART0,
    UART1,
    UART2,
    ADC0,
    CMP0,
    TPM0,
    TPM1,
    TPM2,
    RTC_ALARM,
    RTC_SECONDS,
    PIT,
    I2S0,
    USB_OTG,
    DAC0,
    TSI0,
    MCG,
    LPTMR0,
    // Blank
    PORTA,
    PORTC_D,
}

unsafe impl Nr for Interrupt {
    #[inline(always)]
    fn nr(&self) -> u8 {
        match *self {
            Interrupt::DMA_CHANNEL0 => 0,
            Interrupt::DMA_CHANNEL1 => 1,
            Interrupt::DMA_CHANNEL2 => 2,
            Interrupt::DMA_CHANNEL3 => 3,
            // Skip 4
            Interrupt::FLASH        => 5, // FTFA complete or collision
            Interrupt::LV_DETECT    => 6,
            Interrupt::LLWU         => 7,
            Interrupt::I2C0         => 8,
            Interrupt::I2C1         => 9,
            Interrupt::SPI0         => 10,
            Interrupt::SPI1         => 11,
            Interrupt::UART0        => 12,
            Interrupt::UART1        => 13,
            Interrupt::UART2        => 14,
            Interrupt::ADC0         => 15,
            Interrupt::CMP0         => 16,
            Interrupt::TPM0         => 17,
            Interrupt::TPM1         => 18,
            Interrupt::TPM2         => 19,
            Interrupt::RTC_ALARM    => 20,
            Interrupt::RTC_SECONDS  => 21,
            Interrupt::PIT          => 22,
            Interrupt::I2S0         => 23,
            Interrupt::USB_OTG      => 24,
            Interrupt::DAC0         => 25,
            Interrupt::TSI0         => 26,
            Interrupt::MCG          => 27,
            Interrupt::LPTMR0       => 28,
            // Skip 29
            Interrupt::PORTA        => 30,
            Interrupt::PORTC_D      => 31,
        }
    }
}

#[macro_export]
macro_rules! interrupt {
    ($Name:ident, $handler:path,state: $State:ty = $initial_state:expr) => {
        #[allow(unsafe_code)]
        #[deny(private_no_mangle_fns)]
        #[no_mangle]
        pub unsafe extern "C" fn $Name() {
            static mut STATE: $State = $initial_state;
            let _ = $crate::interrupts::Interrupt::$Name;
            let f: fn(&mut $State) = $handler;
            f(&mut STATE)
        }
    };
    ($Name:ident, $handler:path) => {
        #[allow(unsafe_code)]
        #[deny(private_no_mangle_fns)]
        #[no_mangle]
        pub unsafe extern "C" fn $Name() {
            let _ = $crate::interrupts::Interrupt::$Name;
            let f: fn() = $handler;
            f()
        }
    };
}

#[doc(hidden)]
pub union Vector {
    _handler: unsafe extern "C" fn(),
    _reserved: u32,
}

#[doc(hidden)]
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [Vector; 32] = [
    Vector { _handler: DMA_CHANNEL0 },
    Vector { _handler: DMA_CHANNEL1 },
    Vector { _handler: DMA_CHANNEL2 },
    Vector { _handler: DMA_CHANNEL3 },
    Vector { _reserved: 0 },
    Vector { _handler: FLASH }, // FTFA complete or collision
    Vector { _handler: LV_DETECT },
    Vector { _handler: LLWU },
    Vector { _handler: I2C0 },
    Vector { _handler: I2C1 },
    Vector { _handler: SPI0 },
    Vector { _handler: SPI1 },
    Vector { _handler: UART0 },
    Vector { _handler: UART1 },
    Vector { _handler: UART2 },
    Vector { _handler: ADC0 },
    Vector { _handler: CMP0 },
    Vector { _handler: TPM0 },
    Vector { _handler: TPM1 },
    Vector { _handler: TPM2 },
    Vector { _handler: RTC_ALARM },
    Vector { _handler: RTC_SECONDS },
    Vector { _handler: PIT },
    Vector { _handler: I2S0 },
    Vector { _handler: USB_OTG },
    Vector { _handler: DAC0 },
    Vector { _handler: TSI0 },
    Vector { _handler: MCG },
    Vector { _handler: LPTMR0 },
    Vector { _reserved: 0 },
    Vector { _handler: PORTA },
    Vector { _handler: PORTC_D },
];

extern "C" {
    fn DMA_CHANNEL0();
    fn DMA_CHANNEL1();
    fn DMA_CHANNEL2();
    fn DMA_CHANNEL3();
    fn FLASH();
    fn LV_DETECT();
    fn LLWU();
    fn I2C0();
    fn I2C1();
    fn SPI0();
    fn SPI1();
    fn UART0();
    fn UART1();
    fn UART2();
    fn ADC0();
    fn CMP0();
    fn TPM0();
    fn TPM1();
    fn TPM2();
    fn RTC_ALARM();
    fn RTC_SECONDS();
    fn PIT();
    fn I2S0();
    fn USB_OTG();
    fn DAC0();
    fn TSI0();
    fn MCG();
    fn LPTMR0();
    fn PORTA();
    fn PORTC_D();
}
