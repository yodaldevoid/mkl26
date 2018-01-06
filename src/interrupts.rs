//! Interrupts
use bare_metal::Nr;

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

unsafe impl Nr for Interrupt {
    #[inline(always)]
    fn nr(&self) -> u8 {
        match *self {
            Interrupt::DMA_CHANNEL0     => 0,
            Interrupt::DMA_CHANNEL1     => 1,
            Interrupt::DMA_CHANNEL2     => 2,
            Interrupt::DMA_CHANNEL3     => 3,
            Interrupt::DMA_CHANNEL4     => 4,
            Interrupt::DMA_CHANNEL5     => 5,
            Interrupt::DMA_CHANNEL6     => 6,
            Interrupt::DMA_CHANNEL7     => 7,
            Interrupt::DMA_CHANNEL8     => 8,
            Interrupt::DMA_CHANNEL9     => 9,
            Interrupt::DMA_CHANNEL10    => 10,
            Interrupt::DMA_CHANNEL11    => 11,
            Interrupt::DMA_CHANNEL12    => 12,
            Interrupt::DMA_CHANNEL13    => 13,
            Interrupt::DMA_CHANNEL14    => 14,
            Interrupt::DMA_CHANNEL15    => 15,
            Interrupt::DMA_ERROR0_15    => 16,
            // Skip 17
            Interrupt::FLASH_COMPLETE   => 18,
            Interrupt::FLASH_COLLISION  => 19,
            Interrupt::LV_DETECT        => 20,
            Interrupt::LLWU             => 21,
            Interrupt::WDOG             => 22,
            // Skip 23
            Interrupt::I2C0             => 24,
            Interrupt::I2C1             => 25,
            Interrupt::SPI0             => 26,
            Interrupt::SPI1             => 27,
            // Skip 28
            Interrupt::CAN0_MSG_BUF     => 29,
            Interrupt::CAN0_BUS_OFF     => 30,
            Interrupt::CAN0_ERROR       => 31,
            Interrupt::CAN0_TX_WARN     => 32,
            Interrupt::CAN0_RX_WARN     => 33,
            Interrupt::CAN0_WKUP        => 34,
            Interrupt::I2S0_TX          => 35,
            Interrupt::I2S0_RX          => 36,
            // Skip 37-43
            Interrupt::UART0_LON        => 44,
            Interrupt::UART0_STATUS     => 45,
            Interrupt::UART0_ERROR      => 46,
            Interrupt::UART1_STATUS     => 47,
            Interrupt::UART1_ERROR      => 48,
            Interrupt::UART2_STATUS     => 49,
            Interrupt::UART2_ERROR      => 50,
            // Skip 51-56
            Interrupt::ADC0             => 57,
            Interrupt::ADC1             => 58,
            Interrupt::CMP0             => 59,
            Interrupt::CMP1             => 60,
            Interrupt::CMP2             => 61,
            Interrupt::FTM0             => 62,
            Interrupt::FTM1             => 63,
            Interrupt::FTM2             => 64,
            Interrupt::CMT              => 65,
            Interrupt::RTC_ALARM        => 66,
            Interrupt::RTC_SECONDS      => 67,
            Interrupt::PIT_CHANNEL0     => 68,
            Interrupt::PIT_CHANNEL1     => 69,
            Interrupt::PIT_CHANNEL2     => 70,
            Interrupt::PIT_CHANNEL3     => 71,
            Interrupt::PDB              => 72,
            Interrupt::USB_OTG          => 73,
            Interrupt::USB_CHRG_DETECT  => 74,
            // Skip 75-80
            Interrupt::DAC0             => 81,
            // Skip 82
            Interrupt::TSI              => 83,
            Interrupt::MCG              => 84,
            Interrupt::LP_TIM           => 85,
            // Skip 86
            Interrupt::PORTA            => 87,
            Interrupt::PORTB            => 88,
            Interrupt::PORTC            => 89,
            Interrupt::PORTD            => 90,
            Interrupt::PORTE            => 91,
            // Skip 92-93
            Interrupt::SOFTWARE         => 94,
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
pub static INTERRUPTS: [Option<unsafe extern "C" fn()>; 95] = [
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

global_asm!(
    "
    .thumb_func
    DH_TRAMPOLINE:
        b DEFAULT_HANDLER
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
    .weak DMA_CHANNEL4
    DMA_CHANNEL4 = DH_TRAMPOLINE
    .weak DMA_CHANNEL5
    DMA_CHANNEL5 = DH_TRAMPOLINE
    .weak DMA_CHANNEL6
    DMA_CHANNEL6 = DH_TRAMPOLINE
    .weak DMA_CHANNEL7
    DMA_CHANNEL7 = DH_TRAMPOLINE
    .weak DMA_CHANNEL8
    DMA_CHANNEL8 = DH_TRAMPOLINE
    .weak DMA_CHANNEL9
    DMA_CHANNEL9 = DH_TRAMPOLINE
    .weak DMA_CHANNEL10
    DMA_CHANNEL10 = DH_TRAMPOLINE
    .weak DMA_CHANNEL11
    DMA_CHANNEL11 = DH_TRAMPOLINE
    .weak DMA_CHANNEL12
    DMA_CHANNEL12 = DH_TRAMPOLINE
    .weak DMA_CHANNEL13
    DMA_CHANNEL13 = DH_TRAMPOLINE
    .weak DMA_CHANNEL14
    DMA_CHANNEL14 = DH_TRAMPOLINE
    .weak DMA_CHANNEL15
    DMA_CHANNEL15 = DH_TRAMPOLINE
    .weak DMA_ERROR0_15
    DMA_ERROR0_15 = DH_TRAMPOLINE
    .weak FLASH_COMPLETE
    FLASH_COMPLETE = DH_TRAMPOLINE
    .weak FLASH_COLLISION
    FLASH_COLLISION = DH_TRAMPOLINE
    .weak LV_DETECT
    LV_DETECT = DH_TRAMPOLINE
    .weak LLWU
    LLWU = DH_TRAMPOLINE
    .weak WDOG
    WDOG = DH_TRAMPOLINE
    .weak I2C0
    I2C0 = DH_TRAMPOLINE
    .weak I2C1
    I2C1 = DH_TRAMPOLINE
    .weak SPI0
    SPI0 = DH_TRAMPOLINE
    .weak SPI1
    SPI1 = DH_TRAMPOLINE
    .weak CAN0_MSG_BUF
    CAN0_MSG_BUF = DH_TRAMPOLINE
    .weak CAN0_BUS_OFF
    CAN0_BUS_OFF = DH_TRAMPOLINE
    .weak CAN0_ERROR
    CAN0_ERROR = DH_TRAMPOLINE
    .weak CAN0_TX_WARN
    CAN0_TX_WARN = DH_TRAMPOLINE
    .weak CAN0_RX_WARN
    CAN0_RX_WARN = DH_TRAMPOLINE
    .weak CAN0_WKUP
    CAN0_WKUP = DH_TRAMPOLINE
    .weak I2S0_TX
    I2S0_TX = DH_TRAMPOLINE
    .weak I2S0_RX
    I2S0_RX = DH_TRAMPOLINE
    .weak UART0_LON
    UART0_LON = DH_TRAMPOLINE
    .weak UART0_STATUS
    UART0_STATUS = DH_TRAMPOLINE
    .weak UART0_ERROR
    UART0_ERROR = DH_TRAMPOLINE
    .weak UART1_STATUS
    UART1_STATUS = DH_TRAMPOLINE
    .weak UART1_ERROR
    UART1_ERROR = DH_TRAMPOLINE
    .weak UART2_STATUS
    UART2_STATUS = DH_TRAMPOLINE
    .weak UART2_ERROR
    UART2_ERROR = DH_TRAMPOLINE
    .weak ADC0
    ADC0 = DH_TRAMPOLINE
    .weak ADC1
    ADC1 = DH_TRAMPOLINE
    .weak CMP0
    CMP0 = DH_TRAMPOLINE
    .weak CMP1
    CMP1 = DH_TRAMPOLINE
    .weak CMP2
    CMP2 = DH_TRAMPOLINE
    .weak FTM0
    FTM0 = DH_TRAMPOLINE
    .weak FTM1
    FTM1 = DH_TRAMPOLINE
    .weak FTM2
    FTM2 = DH_TRAMPOLINE
    .weak CMT
    CMT = DH_TRAMPOLINE
    .weak RTC_ALARM
    RTC_ALARM = DH_TRAMPOLINE
    .weak RTC_SECONDS
    RTC_SECONDS = DH_TRAMPOLINE
    .weak PIT_CHANNEL0
    PIT_CHANNEL0 = DH_TRAMPOLINE
    .weak PIT_CHANNEL1
    PIT_CHANNEL1 = DH_TRAMPOLINE
    .weak PIT_CHANNEL2
    PIT_CHANNEL2 = DH_TRAMPOLINE
    .weak PIT_CHANNEL3
    PIT_CHANNEL3 = DH_TRAMPOLINE
    .weak PDB
    PDB = DH_TRAMPOLINE
    .weak USB_OTG
    USB_OTG = DH_TRAMPOLINE
    .weak USB_CHRG_DETECT
    USB_CHRG_DETECT = DH_TRAMPOLINE
    .weak DAC0
    DAC0 = DH_TRAMPOLINE
    .weak TSI
    TSI = DH_TRAMPOLINE
    .weak MCG
    MCG = DH_TRAMPOLINE
    .weak LP_TIM
    LP_TIM = DH_TRAMPOLINE
    .weak PORTA
    PORTA = DH_TRAMPOLINE
    .weak PORTB
    PORTB = DH_TRAMPOLINE
    .weak PORTC
    PORTC = DH_TRAMPOLINE
    .weak PORTD
    PORTD = DH_TRAMPOLINE
    .weak PORTE
    PORTE = DH_TRAMPOLINE
    .weak SOFTWARE
    SOFTWARE = DH_TRAMPOLINE
    "
);

extern "C" {
    fn DMA_CHANNEL0();
    fn DMA_CHANNEL1();
    fn DMA_CHANNEL2();
    fn DMA_CHANNEL3();
    fn DMA_CHANNEL4();
    fn DMA_CHANNEL5();
    fn DMA_CHANNEL6();
    fn DMA_CHANNEL7();
    fn DMA_CHANNEL8();
    fn DMA_CHANNEL9();
    fn DMA_CHANNEL10();
    fn DMA_CHANNEL11();
    fn DMA_CHANNEL12();
    fn DMA_CHANNEL13();
    fn DMA_CHANNEL14();
    fn DMA_CHANNEL15();
    fn DMA_ERROR0_15();
    fn FLASH_COMPLETE();
    fn FLASH_COLLISION();
    fn LV_DETECT();
    fn LLWU();
    fn WDOG();
    fn I2C0();
    fn I2C1();
    fn SPI0();
    fn SPI1();
    fn CAN0_MSG_BUF();
    fn CAN0_BUS_OFF();
    fn CAN0_ERROR();
    fn CAN0_TX_WARN();
    fn CAN0_RX_WARN();
    fn CAN0_WKUP();
    fn I2S0_TX();
    fn I2S0_RX();
    fn UART0_LON();
    fn UART0_STATUS();
    fn UART0_ERROR();
    fn UART1_STATUS();
    fn UART1_ERROR();
    fn UART2_STATUS();
    fn UART2_ERROR();
    fn ADC0();
    fn ADC1();
    fn CMP0();
    fn CMP1();
    fn CMP2();
    fn FTM0();
    fn FTM1();
    fn FTM2();
    fn CMT();
    fn RTC_ALARM();
    fn RTC_SECONDS();
    fn PIT_CHANNEL0();
    fn PIT_CHANNEL1();
    fn PIT_CHANNEL2();
    fn PIT_CHANNEL3();
    fn PDB();
    fn USB_OTG();
    fn USB_CHRG_DETECT();
    fn DAC0();
    fn TSI();
    fn MCG();
    fn LP_TIM();
    fn PORTA();
    fn PORTB();
    fn PORTC();
    fn PORTD();
    fn PORTE();
    fn SOFTWARE();
}
