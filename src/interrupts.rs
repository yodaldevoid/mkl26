//! Interrupts
use bare_metal::Nr;

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
    ($NAME: ident, $path: path, locals: { $($lvar: ident: $lty :ident = $lval: expr;)* }) => {
        #[allow(non_snake_case)]
        mod $NAME {
            pub struct Locals {
                $(pub $lvar : $lty,)*
            }
        }
        #[allow(non_snake_case)]
        #[no_mangle]
        pub extern "C" fn $NAME () {
            let _ = $crate::interrupts::Interrupt::$NAME;
            static mut LOCALS: self::$NAME::Locals = self::$NAME::Locals {
                $($lvar: $lval,)*
            };
            let f: fn(&mut self::$NAME::Locals) = $path;
            f(unsafe {&mut LOCALS});
        }
    };
    ($ NAME: ident, $path: path) => {
        #[allow(non_snake_case)]
        #[no_mangle]
        pub extern "C" fn $ NAME () {
            let _ = $crate::interrupts::Interrupt::$NAME;
            let f: fn() = $path;
            f();
        }
    }
}

#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static INTERRUPTS: [Option<unsafe extern "C" fn()>; 32] = [
    Some(DMA_CHANNEL0),
    Some(DMA_CHANNEL1),
    Some(DMA_CHANNEL2),
    Some(DMA_CHANNEL3),
    None,
    Some(FLASH), // FTFA complete or collision
    Some(LV_DETECT),
    Some(LLWU),
    Some(I2C0),
    Some(I2C1),
    Some(SPI0),
    Some(SPI1),
    Some(UART0),
    Some(UART1),
    Some(UART2),
    Some(ADC0),
    Some(CMP0),
    Some(TPM0),
    Some(TPM1),
    Some(TPM2),
    Some(RTC_ALARM),
    Some(RTC_SECONDS),
    Some(PIT),
    Some(I2S0),
    Some(USB_OTG),
    Some(DAC0),
    Some(TSI0),
    Some(MCG),
    Some(LPTMR0),
    None,
    Some(PORTA),
    Some(PORTC_D),
];

global_asm!(
    "
    .thumb_func
    DH_TRAMPOLINE:
        ldr r0,=DEFAULT_HANDLER
        bx r0
    "
);

global_asm!(
    "
    .weak DMA_CHANNEL0
    DMA_CHANNEL0 = DH_TRAMPOLINE
    .weak DMA_CHANNEL1
    DMA_CHANNEL1 = DH_TRAMPOLINE
    .weak DMA_CHANNEL2
    DMA_CHANNEL2 = DH_TRAMPOLINE
    .weak DMA_CHANNEL3
    DMA_CHANNEL3 = DH_TRAMPOLINE
    .weak FLASH
    FLASH = DH_TRAMPOLINE
    .weak LV_DETECT
    LV_DETECT = DH_TRAMPOLINE
    .weak LLWU
    LLWU = DH_TRAMPOLINE
    .weak I2C0
    I2C0 = DH_TRAMPOLINE
    .weak I2C1
    I2C1 = DH_TRAMPOLINE
    .weak SPI0
    SPI0 = DH_TRAMPOLINE
    .weak SPI1
    SPI1 = DH_TRAMPOLINE
    .weak UART0
    UART0 = DH_TRAMPOLINE
    .weak UART1
    UART1 = DH_TRAMPOLINE
    .weak UART2
    UART2 = DH_TRAMPOLINE
    .weak ADC0
    ADC0 = DH_TRAMPOLINE
    .weak CMP0
    CMP0 = DH_TRAMPOLINE
    .weak TPM0
    TPM0 = DH_TRAMPOLINE
    .weak TPM1
    TPM1 = DH_TRAMPOLINE
    .weak TPM2
    TPM2 = DH_TRAMPOLINE
    .weak RTC_ALARM
    RTC_ALARM = DH_TRAMPOLINE
    .weak RTC_SECONDS
    RTC_SECONDS = DH_TRAMPOLINE
    .weak PIT
    PIT = DH_TRAMPOLINE
    .weak I2S0
    I2S0 = DH_TRAMPOLINE
    .weak USB_OTG
    USB_OTG = DH_TRAMPOLINE
    .weak DAC0
    DAC0 = DH_TRAMPOLINE
    .weak TSI0
    TSI0 = DH_TRAMPOLINE
    .weak MCG
    MCG = DH_TRAMPOLINE
    .weak LPTMR0
    LPTMR0 = DH_TRAMPOLINE
    .weak PORTA
    PORTA = DH_TRAMPOLINE
    .weak PORTC_D
    PORTC_D = DH_TRAMPOLINE
    "
);

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
