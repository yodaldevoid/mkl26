//! Interrupts

use exceptions::default_handler;

#[allow(non_camel_case_types)]
pub enum Interrupt {
    DMA_CHANNEL0,
    DMA_CHANNEL1,
    DMA_CHANNEL2,
    DMA_CHANNEL3,
    DMA_CHANNEL4,
    DMA_CHANNEL5,
    DMA_CHANNEL6,
    DMA_CHANNEL7,
    DMA_CHANNEL8,
    DMA_CHANNEL9,
    DMA_CHANNEL10,
    DMA_CHANNEL11,
    DMA_CHANNEL12,
    DMA_CHANNEL13,
    DMA_CHANNEL14,
    DMA_CHANNEL15,
    DMA_ERROR0_15,
    FLASH_COMPLETE,
    FLASH_COLLISION,
    LV_DETECT,
    LLWU,
    WDOG,
    I2C0,
    I2C1,
    SPI0,
    SPI1,
    CAN0_MSG_BUF,
    CAN0_BUS_OFF,
    CAN0_ERROR,
    CAN0_TX_WARN,
    CAN0_RX_WARN,
    CAN0_WKUP,
    I2S0_TX,
    I2S0_RX,
    UART0_LON,
    UART0_STATUS,
    UART0_ERROR,
    UART1_STATUS,
    UART1_ERROR,
    UART2_STATUS,
    UART2_ERROR,
    ADC0,
    ADC1,
    CMP0,
    CMP1,
    CMP2,
    FTM0,
    FTM1,
    FTM2,
    CMT,
    RTC_ALARM,
    RTC_SECONDS,
    PIT_CHANNEL0,
    PIT_CHANNEL1,
    PIT_CHANNEL2,
    PIT_CHANNEL3,
    PDB,
    USB_OTG,
    USB_CHRG_DETECT,
    DAC0,
    TSI,
    MCG,
    LP_TIM,
    PORTA,
    PORTB,
    PORTC,
    PORTD,
    PORTE,
    SOFTWARE,
}

/// Interrupt number
pub unsafe trait Nr {
    /// Returns the number associated with an interrupt
    fn nr(&self) -> u8;
}

unsafe impl Nr for Interrupt {
    #[inline(always)]
    fn nr(&self) -> u8 {
        match *self {
            Interrupt::DMA_CHANNEL0     => 16,
            Interrupt::DMA_CHANNEL1     => 17,
            Interrupt::DMA_CHANNEL2     => 18,
            Interrupt::DMA_CHANNEL3     => 19,
            Interrupt::DMA_CHANNEL4     => 20,
            Interrupt::DMA_CHANNEL5     => 21,
            Interrupt::DMA_CHANNEL6     => 22,
            Interrupt::DMA_CHANNEL7     => 23,
            Interrupt::DMA_CHANNEL8     => 24,
            Interrupt::DMA_CHANNEL9     => 25,
            Interrupt::DMA_CHANNEL10    => 26,
            Interrupt::DMA_CHANNEL11    => 27,
            Interrupt::DMA_CHANNEL12    => 28,
            Interrupt::DMA_CHANNEL13    => 29,
            Interrupt::DMA_CHANNEL14    => 30,
            Interrupt::DMA_CHANNEL15    => 31,
            Interrupt::DMA_ERROR0_15    => 32,
            Interrupt::FLASH_COMPLETE   => 34,
            Interrupt::FLASH_COLLISION  => 35,
            Interrupt::LV_DETECT        => 36,
            Interrupt::LLWU             => 37,
            Interrupt::WDOG             => 38,
            Interrupt::I2C0             => 40,
            Interrupt::I2C1             => 41,
            Interrupt::SPI0             => 42,
            Interrupt::SPI1             => 43,
            Interrupt::CAN0_MSG_BUF     => 45,
            Interrupt::CAN0_BUS_OFF     => 46,
            Interrupt::CAN0_ERROR       => 47,
            Interrupt::CAN0_TX_WARN     => 48,
            Interrupt::CAN0_RX_WARN     => 49,
            Interrupt::CAN0_WKUP        => 50,
            Interrupt::I2S0_TX          => 51,
            Interrupt::I2S0_RX          => 52,
            Interrupt::UART0_LON        => 60,
            Interrupt::UART0_STATUS     => 61,
            Interrupt::UART0_ERROR      => 62,
            Interrupt::UART1_STATUS     => 63,
            Interrupt::UART1_ERROR      => 64,
            Interrupt::UART2_STATUS     => 65,
            Interrupt::UART2_ERROR      => 66,
            Interrupt::ADC0             => 73,
            Interrupt::ADC1             => 74,
            Interrupt::CMP0             => 75,
            Interrupt::CMP1             => 76,
            Interrupt::CMP2             => 77,
            Interrupt::FTM0             => 78,
            Interrupt::FTM1             => 79,
            Interrupt::FTM2             => 80,
            Interrupt::CMT              => 81,
            Interrupt::RTC_ALARM        => 82,
            Interrupt::RTC_SECONDS      => 83,
            Interrupt::PIT_CHANNEL0     => 84,
            Interrupt::PIT_CHANNEL1     => 85,
            Interrupt::PIT_CHANNEL2     => 86,
            Interrupt::PIT_CHANNEL3     => 87,
            Interrupt::PDB              => 88,
            Interrupt::USB_OTG          => 89,
            Interrupt::USB_CHRG_DETECT  => 90,
            Interrupt::DAC0             => 97,
            Interrupt::TSI              => 99,
            Interrupt::MCG              => 100,
            Interrupt::LP_TIM           => 101,
            Interrupt::PORTA            => 103,
            Interrupt::PORTB            => 104,
            Interrupt::PORTC            => 105,
            Interrupt::PORTD            => 106,
            Interrupt::PORTE            => 107,
            Interrupt::SOFTWARE         => 110,
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
            let _ = $crate::interrupt::Interrupt::$NAME;
            static mut LOCALS: self::$NAME::Locals = self::$NAME::Locals {
                $($lvar: $lval,)*
            };
            let f: fn(&mut self::$NAME::Locals) = $path;
            f(unsafe {&mut LOCALS});
        }
    };
    ($ NAME: ident, $path: path) => {
        # [allow(non_snake_case)]
        # [no_mangle] pub extern "C" fn $ NAME () {
            let _ = $crate::interrupt::Interrupt::$NAME ;
            let f: fn() = $path;
            f();
        }
    }
}

#[link_section = ".vectors.interrupts"]
#[no_mangle]
pub static INTERRUPTS: [Option<unsafe extern fn()>; 95] = [
    Some(DMA_CHANNEL0),
    Some(DMA_CHANNEL1),
    Some(DMA_CHANNEL2),
    Some(DMA_CHANNEL3),
    Some(DMA_CHANNEL4),
    Some(DMA_CHANNEL5),
    Some(DMA_CHANNEL6),
    Some(DMA_CHANNEL7),
    Some(DMA_CHANNEL8),
    Some(DMA_CHANNEL9),
    Some(DMA_CHANNEL10),
    Some(DMA_CHANNEL11),
    Some(DMA_CHANNEL12),
    Some(DMA_CHANNEL13),
    Some(DMA_CHANNEL14),
    Some(DMA_CHANNEL15),
    Some(DMA_ERROR0_15),
    None,
    Some(FLASH_COMPLETE),
    Some(FLASH_COLLISION),
    Some(LV_DETECT),
    Some(LLWU),
    Some(WDOG),
    None,
    Some(I2C0),
    Some(I2C1),
    Some(SPI0),
    Some(SPI1),
    None,
    Some(CAN0_MSG_BUF),
    Some(CAN0_BUS_OFF),
    Some(CAN0_ERROR),
    Some(CAN0_TX_WARN),
    Some(CAN0_RX_WARN),
    Some(CAN0_WKUP),
    Some(I2S0_TX),
    Some(I2S0_RX),
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    Some(UART0_LON),
    Some(UART0_STATUS),
    Some(UART0_ERROR),
    Some(UART1_STATUS),
    Some(UART1_ERROR),
    Some(UART2_STATUS),
    Some(UART2_ERROR),
    None,
    None,
    None,
    None,
    None,
    None,
    Some(ADC0),
    Some(ADC1),
    Some(CMP0),
    Some(CMP1),
    Some(CMP2),
    Some(FTM0),
    Some(FTM1),
    Some(FTM2),
    Some(CMT),
    Some(RTC_ALARM),
    Some(RTC_SECONDS),
    Some(PIT_CHANNEL0),
    Some(PIT_CHANNEL1),
    Some(PIT_CHANNEL2),
    Some(PIT_CHANNEL3),
    Some(PDB),
    Some(USB_OTG),
    Some(USB_CHRG_DETECT),
    None,
    None,
    None,
    None,
    None,
    None,
    Some(DAC0),
    None,
    Some(TSI),
    Some(MCG),
    Some(LP_TIM),
    None,
    Some(PORTA),
    Some(PORTB),
    Some(PORTC),
    Some(PORTD),
    Some(PORTE),
    None,
    None,
    Some(SOFTWARE),
];

#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL0() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL1() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL2() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL3() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL4() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL5() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL6() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL7() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL8() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL9() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL10() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL11() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL12() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL13() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL14() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_CHANNEL15() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DMA_ERROR0_15() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn FLASH_COMPLETE() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn FLASH_COLLISION() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn LV_DETECT() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn LLWU() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn WDOG() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn I2C0() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn I2C1() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn SPI0() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn SPI1() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CAN0_MSG_BUF() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CAN0_BUS_OFF() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CAN0_ERROR() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CAN0_TX_WARN() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CAN0_RX_WARN() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CAN0_WKUP() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn I2S0_TX() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn I2S0_RX() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn UART0_LON() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn UART0_STATUS() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn UART0_ERROR() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn UART1_STATUS() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn UART1_ERROR() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn UART2_STATUS() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn UART2_ERROR() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn ADC0() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn ADC1() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CMP0() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CMP1() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CMP2() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn FTM0() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn FTM1() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn FTM2() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn CMT() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn RTC_ALARM() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn RTC_SECONDS() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PIT_CHANNEL0() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PIT_CHANNEL1() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PIT_CHANNEL2() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PIT_CHANNEL3() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PDB() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn USB_OTG() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn USB_CHRG_DETECT() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn DAC0() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn TSI() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn MCG() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn LP_TIM() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PORTA() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PORTB() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PORTC() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PORTD() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn PORTE() { default_handler() }
#[linkage = "weak"]
#[no_mangle]
extern fn SOFTWARE() { default_handler() }
