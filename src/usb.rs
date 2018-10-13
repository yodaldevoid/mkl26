use core::cell::RefCell;
use core::mem::size_of;
use core::ops::{Index, IndexMut};
use core::ptr::{NonNull, null_mut};
use core::slice;

use bit_field::BitField;
use cortex_m::interrupt::{self, CriticalSection, Mutex};
use cortex_m::peripheral::NVIC;
use volatile_register::{RO, RW};

use interrupts::Interrupt;
use sim::ClockGate;

const USB0_ADDR: usize = 0x4007_2000;

// These are used to pad the registers to be 4 byte aligned.
// repr(align(32)) is not used as that cannot be used in packed structs.
#[repr(C, packed)]
struct UsbRO(RO<u8>, [u8; 3]);

#[repr(C, packed)]
struct UsbRW(RW<u8>, [u8; 3]);

impl UsbRO {
    fn read(&self) -> u8 {
        self.0.read()
    }
}

impl UsbRW {
    fn read(&self) -> u8 {
        self.0.read()
    }

    unsafe fn write(&self, value: u8) {
        self.0.write(value)
    }

    unsafe fn modify<F>(&self, f: F)
    where
        F: FnOnce(u8) -> u8
    {
        self.0.modify(f);
    }
}

#[repr(C,packed)]
struct UsbRegs {
    perid:      UsbRO,
    idcomp:     UsbRO,
    rev:        UsbRO,
    addinfo:    UsbRO,

    otgistat:   UsbRW,
    otgicr:     UsbRW,
    otgstat:    UsbRW,
    otgctl:     UsbRW,

    _pad8: [u32; 24],

    istat:      UsbRW,
    inten:      UsbRW,
    errstat:    UsbRW,
    erren:      UsbRW,
    stat:       UsbRO,
    ctl:        UsbRW,
    addr:       UsbRW,

    bdtpage1:   UsbRW,
    frmnuml:    UsbRW,
    frmnumh:    UsbRW,
    token:      UsbRW,
    softhld:    UsbRW,
    bdtpage2:   UsbRW,
    bdtpage3:   UsbRW,

    _pad23: [u32; 2],

    endpt0:     UsbRW,
    endpt1:     UsbRW,
    endpt2:     UsbRW,
    endpt3:     UsbRW,
    endpt4:     UsbRW,
    endpt5:     UsbRW,
    endpt6:     UsbRW,
    endpt7:     UsbRW,
    endpt8:     UsbRW,
    endpt9:     UsbRW,
    endpt10:    UsbRW,
    endpt11:    UsbRW,
    endpt12:    UsbRW,
    endpt13:    UsbRW,
    endpt14:    UsbRW,
    endpt15:    UsbRW,

    usbctrl:    UsbRW,

    observe:    UsbRO,
    control:    UsbRW,

    usbtrc0:    UsbRW,

    _pad44: u32,

    usbfrmadjust: UsbRW,
}

impl UsbRegs {
    unsafe fn parse_istat(&mut self) -> InterruptFlags {
        let istat = self.istat.read();
        let flags = InterruptFlags::from_bits_truncate(istat);
        flags
    }
}

pub struct Usb {
    reg: &'static mut UsbRegs,
    _gate: ClockGate,
    bus: u8,
}

// TODO: should we even pass in the bus variable (or use a const generic) as
// there is only one USB peripheral?
/// The USB peripheral requires a minimum system clock frequency of 20 MHz and
/// a module clock frequency of 48 Mhz.
impl Usb {
    pub unsafe fn new(
        bus: u8,
        nvic: &mut NVIC,
        gate: ClockGate) -> Result<Usb, ()> {
        let reg = match bus {
            0 => &mut *(USB0_ADDR as *mut UsbRegs),
            _ => unimplemented!()
        };

        interrupt::free(|cs| {
            match bus {
                0 => *USB0_STATE.borrow(cs).borrow_mut() = Some(IsrState::new()),
                _ => unreachable!(),
            };
        });

        // init usb serialnumber

        // zero out bdts
        let bdt_addr = interrupt::free(|cs| {
            match bus {
                0 => {
                    for bdt in USB0_BDTS.borrow(cs).borrow_mut().0.iter_mut() {
                        bdt.desc = 0;
                        bdt.addr = null_mut();
                    }

                    &*USB0_BDTS.borrow(cs).borrow() as *const Tables
                }
                _ => unreachable!(),
            }
        });

        // reset USB module - commented out in example code
        // reg.usbtrc0.modify(|mut usbtrc0| {
        //     usbtrc0.set_bit(7, true);
        //     usbtrc0
        // });
        // while reg.usbtrc0.read().get_bit(7) {}

        // set desc table base addr
        reg.bdtpage1.write((bdt_addr as u64 >> 8) as u8);
        reg.bdtpage2.write((bdt_addr as u64 >> 16) as u8);
        reg.bdtpage3.write((bdt_addr as u64 >> 32) as u8);

        // clear all ISR flags
        reg.istat.write(0xFF);
        reg.errstat.write(0xFF);
        reg.otgistat.write(0xFF);

        // set an undocumented bit - commented out in example code
        // reg.usbtrc0.modify(|mut usbtrc0| {
        //     usbtrc0.set_bit(6, true);
        //     usbtrc0
        // });

        // enable USB
        reg.ctl.modify(|mut ctl| {
            ctl.set_bit(0, true);
            ctl
        });
        reg.usbctrl.write(0);

        // enable reset interrupt
        reg.inten.modify(|mut inten| {
            inten.set_bit(0, true);
            inten
        });

        // enable interrupt in NVIC
        let interrupt = match bus {
            0 => Interrupt::USB_OTG,
            _ => unreachable!(),
        };
        interrupt::free(|_| {
            nvic.enable(interrupt);
        });

        // enable d+ pullup
        // OK to just write and not modify as everything else is read only.
        reg.control.write(1 << 4);

        Ok(Usb { reg, _gate: gate, bus: bus })
    }

    fn init_serialnumber() {
        // disable IRQs
        // read last 4 bytes in "program once" field
        // ftfa.fstat.write(0x70); // clear RDOLERR, ACCERR, and FPVIOL
        // ftfa.fccob0 = 0x41; // "read once" command
        // ftfa.fccob1 = 0x0F;
        // ftfa.fstat.write(0x80); // clear CCIF to start command
        // while !ftfa.fstat.read().get_bit(7) {}
        // num = ftfa.fccob7.read() as u32;
        // enable IRQs

        // add extra zero to work around OS-X CDC-ACM driver bug
        // let num = if num < 10_000_000 {
        //     num * 10
        // } else {
        //     num
        // }
        // convert num to string with a max of 10 chars
        // move string to global
        // set global len to len * 2 + 2
    }
}

struct CdcDataInterface {
    interface: u8,
    line_coding: [u32; 2],
    // TODO: split into enums
    line_rtsdtr: u8,
    transmit_flush_timer: u8,
    usb_serial_flush_callback: Option<fn()>,
}

static mut CDC_DATA_INTERFACE: Mutex<RefCell<Option<CdcDataInterface>>> = Mutex::new(RefCell::new(None));

#[derive(Clone, Copy, PartialEq)]
enum Data {
    Data0 = 0,
    Data1 = 1,
}

#[derive(PartialEq)]
enum TokenPid {
    // Device mode
    Out = 0x01,
    In = 0x09,
    Setup = 0x0D,

    // Host mode
    Data0 = 0x03,
    Data1 = 0x0B,
    Ack = 0x02,
    Stall = 0x0E,
    Nak = 0x0A,
    Timeout = 0x00,
    DataError = 0x0F,

    Unknown,
}

#[derive(Clone, Copy)]
#[repr(C,packed)]
struct Bdt {
    desc: u32,
    addr: *mut u8,
}

unsafe impl Send for Bdt {}

impl Bdt {
    fn set_desc(&mut self, size: usize, data: Data) {
        let mut desc = 0;
        desc.set_bit(7, true); // OWN
        desc.set_bit(3, true); // DTS
        desc.set_bit(6, data == Data::Data1);
        desc.set_bits(16..26, size as u32);
        self.desc = desc;
    }

    #[inline(always)]
    pub fn byte_count(&self) -> u16 {
        unsafe {
            self.desc.get_bits(16..26) as u16
        }
    }

    #[inline(always)]
    pub fn is_processor_owned(&self) -> bool {
        unsafe {
            self.desc.get_bit(7)
        }
    }

    #[inline(always)]
    pub fn set_owned(&mut self, val: bool) {
        unsafe {
            self.desc.set_bit(7, val);
        }
    }

    #[inline(always)]
    pub fn data_num(&self) -> u32 {
        unsafe {
            self.desc.get_bit(6) as u32
        }
    }

    #[inline(always)]
    pub fn set_data_num(&mut self, val: bool) {
        unsafe {
            self.desc.set_bit(6, val);
        }
    }

    #[inline(always)]
    pub fn keep(&self) -> bool {
        unsafe {
            self.desc.get_bit(5)
        }
    }

    #[inline(always)]
    pub fn set_keep(&mut self, val: bool) {
        unsafe {
            self.desc.set_bit(5, val);
        }
    }

    #[inline(always)]
    pub fn no_increment(&self) -> bool {
        unsafe {
            self.desc.get_bit(4)
        }
    }

    #[inline(always)]
    pub fn set_no_increments(&mut self, val: bool) {
        unsafe {
            self.desc.set_bit(4, val);
        }
    }

    #[inline(always)]
    pub fn dts_enabled(&self) -> bool {
        unsafe {
            self.desc.get_bit(3)
        }
    }

    #[inline(always)]
    pub fn set_dts_enabled(&mut self, val: bool) {
        unsafe {
            self.desc.set_bit(3, val);
        }
    }

    #[inline(always)]
    pub fn stall(&self) -> bool {
        unsafe {
            self.desc.get_bit(2)
        }
    }

    #[inline(always)]
    pub fn set_stall(&mut self, val: bool) {
        unsafe {
            self.desc.set_bit(2, val);
        }
    }

    #[inline(always)]
    pub fn token_pid(&self) -> TokenPid {
        match unsafe { self.desc.get_bits(2..6) } {
            0x01 => TokenPid::Out,
            0x09 => TokenPid::In,
            0x0D => TokenPid::Setup,
            0x03 => TokenPid::Data0,
            0x0B => TokenPid::Data1,
            0x02 => TokenPid::Ack,
            0x0E => TokenPid::Stall,
            0x0A => TokenPid::Nak,
            0x00 => TokenPid::Timeout,
            0x0F => TokenPid::DataError,
            _ => TokenPid::Unknown,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Direction {
    Rx = 0,
    Tx = 1,
}

#[derive(Clone, Copy, PartialEq)]
enum Pair {
    Even = 0,
    Odd = 1,
}

// TODO: const generic to specify the size
#[repr(align(512))]
struct Tables([Bdt; 64]);

impl Tables {
    const fn new() -> Tables {
        Tables([
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
            Bdt { desc: 0, addr: null_mut() },
        ])
    }

    // TODO: make safe?
    fn stat_to_bdt(&mut self, stat: u8) -> &mut Bdt {
        &mut self.0[(stat >> 2) as usize]
    }
}

impl Index<(u8, Direction, Pair)> for Tables {
    type Output = Bdt;

    fn index(&self, (endpoint, dir, pair): (u8, Direction, Pair)) -> &Bdt {
        let index = ((endpoint as usize) << 2) | ((dir as usize) << 1) | (pair as usize);
        &self.0[index]
    }
}

impl IndexMut<(u8, Direction, Pair)> for Tables {
    fn index_mut(&mut self, (endpoint, dir, pair): (u8, Direction, Pair)) -> &mut Bdt {
        let index = ((endpoint as usize) << 2) | ((dir as usize) << 1) | (pair as usize);
        &mut self.0[index]
    }
}

static USB0_BDTS: Mutex<RefCell<Tables>> = Mutex::new(RefCell::new(Tables::new()));

#[derive(Clone, Copy)]
#[repr(C,packed)]
struct Packet {
    len: u16,
    index: u16,
    next: Option<NonNull<Packet>>,
    buf: [u8; Packet::MAX_LEN],
}

impl Packet {
    const MAX_LEN: usize = 64;

    const fn new() -> Packet {
        Packet {
            len: 0,
            index: 0,
            next: None,
            buf: [0; Packet::MAX_LEN],
        }
    }
}

// TODO: const generics
struct PacketArena {
    arr: [Packet; PacketArena::NUM_USB_BUFFERS],
    used: u32,
}

impl PacketArena {
    const NUM_USB_BUFFERS: usize = 32;

    const fn new() -> PacketArena {
        PacketArena {
            arr: [Packet::new(); PacketArena::NUM_USB_BUFFERS],
            used: u32::max_value(),
        }
    }

    fn alloc(&mut self) -> Option<NonNull<Packet>> {
        interrupt::free(|_| {
            let n = self.used.leading_zeros() as usize;
            if n >= PacketArena::NUM_USB_BUFFERS {
                None
            } else {
                self.used &= !(0x8000_0000 >> n);
                let mut packet = self.arr[n];
                packet.len = 0;
                packet.index = 0;
                Some(unsafe { NonNull::new_unchecked(&mut packet as *mut Packet) })
            }
        })
    }

    fn free(&mut self, packet: NonNull<Packet>) {
        let start = self.arr.as_mut_ptr() as usize;
        let n = (packet.as_ptr() as usize - start) / size_of::<Packet>();
        if n < PacketArena::NUM_USB_BUFFERS {
            // TODO: RX memory needed

            interrupt::free(|_| {
                self.used |= 0x8000_0000 >> n;
            })
        }
    }
}

static mut PACKET_HEAP: PacketArena = PacketArena::new();

bitflags! {
    struct InterruptFlags: u8 {
        const USB_RESET = 0x01;
        const ERROR = 0x02;
        const SOF_THRESHOLD_OK = 0x04;
        const TOKEN_DONE = 0x08;
        const SLEEP = 0x10;
        const RESUME = 0x20;
        const ATTACH = 0x40;
        const STALL = 0x80;
    }
}

#[derive(Clone, Copy)]
enum TxState {
    BothFreeEvenFirst,
    BothFreeOddFirst,
    EvenFree,
    OddFree,
    NoneFreeEvenFirst,
    NoneFreeOddFirst,
}

const NUM_ENDPOINTS: usize = 16;
const EP0_SIZE: usize = 64;

struct IsrState {
    configuration: u8, // was volatile global
    reboot_timer: u8, // was volatile global
    cdc_transmit_flush_timer: u8, // was volatile global
    seremu_transmit_flush_timer: u8, // was volatile global

    rx_first: [Option<NonNull<Packet>>; NUM_ENDPOINTS], // from outside of the ISRs
    rx_last:  [Option<NonNull<Packet>>; NUM_ENDPOINTS],
    tx_first: [Option<NonNull<Packet>>; NUM_ENDPOINTS], // from outside of the ISRs
    tx_last:  [Option<NonNull<Packet>>; NUM_ENDPOINTS], // from outside of the ISRs
    rx_byte_count: [u16; NUM_ENDPOINTS], // from outside of the ISRs

    tx_state: [TxState; NUM_ENDPOINTS],

    ep0_rx0_buf: [u8; EP0_SIZE],
    ep0_rx1_buf: [u8; EP0_SIZE],
    ep0_tx_ptr: Option<(NonNull<u8>, usize)>,
    ep0_tx_bdt_bank: Pair,
    ep0_tx_data_toggle: Data,

    reply_buffer: [u8; 8],

    #[cfg(feature = "usb-seremu")]
    seremu_interface: u8, // TODO: move into Seremu struct
}

impl IsrState {
    fn new() -> IsrState {
        IsrState {
            configuration: 0,
            reboot_timer: 0,
            cdc_transmit_flush_timer: 0,
            seremu_transmit_flush_timer: 0,

            rx_first: [None; NUM_ENDPOINTS], // from outside of the ISRs
            rx_last:  [None; NUM_ENDPOINTS],
            tx_first: [None; NUM_ENDPOINTS], // from outside of the ISRs
            tx_last:  [None; NUM_ENDPOINTS], // from outside of the ISRs
            rx_byte_count: [0; NUM_ENDPOINTS], // from outside of the ISRs

            tx_state: [TxState::BothFreeEvenFirst; NUM_ENDPOINTS],

            ep0_rx0_buf: [0; EP0_SIZE],
            ep0_rx1_buf: [0; EP0_SIZE],
            ep0_tx_ptr: None,
            ep0_tx_bdt_bank: Pair::Even,
            ep0_tx_data_toggle: Data::Data0,

            reply_buffer: [0; 8],

            #[cfg(feature = "usb-seremu")]
            seremu_interface: 0,
        }
    }
}

// Shakey, but we only have one thread and this will always be in a CriticalSection guarded Mutex
unsafe impl Send for IsrState {}

static USB0_STATE: Mutex<RefCell<Option<IsrState>>> = Mutex::new(RefCell::new(None));

interrupt!(USB_OTG, isr0);
fn isr0() {
    unsafe {
        interrupt::free(|cs| {
            let reg = &mut *(USB0_ADDR as *mut UsbRegs);
            let mut state = USB0_STATE.borrow(cs).borrow_mut();
            let state = state.as_mut().expect("USB0_STATE uninitialized");

            isr(cs, reg, state);
        });
    }
}

unsafe fn isr(cs: &CriticalSection, reg: &mut UsbRegs, state: &mut IsrState) {
    loop {
        let status = reg.parse_istat();

        if status.contains(InterruptFlags::SOF_THRESHOLD_OK) {
            if state.configuration != 0 {
                if state.reboot_timer != 0 {
                    state.reboot_timer -= 1;
                    if state.reboot_timer == 0 {
                        // TODO: reboot
                    }
                }

                if state.cdc_transmit_flush_timer != 0 {
                    state.cdc_transmit_flush_timer -= 1;
                    if state.cdc_transmit_flush_timer == 0 {
                        // TODO: serial_flush_callback
                    }
                }

                if state.seremu_transmit_flush_timer != 0 {
                    state.seremu_transmit_flush_timer -= 1;
                    if state.seremu_transmit_flush_timer == 0 {
                        // TODO: serenmu_flush_callback
                    }
                }

                // TODO: MIDI interface
                // TODO: flightsim interface
                // TODO: multitouch interface
            }
            reg.istat.write(InterruptFlags::SOF_THRESHOLD_OK.bits());
        }

        if status.contains(InterruptFlags::TOKEN_DONE) {
            let stat = reg.stat.read();
            let endpoint = (stat >> 4) as usize;
            if endpoint == 0 {
                usb_control(cs, reg, state, stat);
            } else {
                let mut tables = USB0_BDTS.borrow(cs).borrow_mut();
                let bdt = tables.stat_to_bdt(stat);
                let mut packet = NonNull::new_unchecked(bdt.addr.sub(8) as *mut Packet);
                let endpoint = endpoint - 1;

                // TODO: Audio interface

                if stat.get_bit(3) { // transmit
                    PACKET_HEAP.free(packet);
                    if let Some(mut packet) = state.tx_first[endpoint] {
                        let packet = packet.as_mut();
                        state.tx_first[endpoint] = packet.next;
                        bdt.addr = packet.buf.as_mut_ptr();
                        match state.tx_state[endpoint] {
                            TxState::BothFreeEvenFirst => state.tx_state[endpoint] = TxState::OddFree,
                            TxState::BothFreeOddFirst => state.tx_state[endpoint] = TxState::EvenFree,
                            TxState::EvenFree => state.tx_state[endpoint] = TxState::NoneFreeOddFirst,
                            TxState::OddFree => state.tx_state[endpoint] = TxState::NoneFreeEvenFirst,
                            _ => {}
                        }
                        let data = if (bdt as *mut Bdt as u32).get_bit(3) {
                            Data::Data1
                        } else {
                            Data::Data0
                        };
                        bdt.set_desc(
                            packet.len as usize,
                            data,
                        );
                    } else {
                        match state.tx_state[endpoint] {
                            TxState::BothFreeEvenFirst | TxState::BothFreeOddFirst => {}
                            TxState::EvenFree => state.tx_state[endpoint] = TxState::BothFreeEvenFirst,
                            TxState::OddFree => state.tx_state[endpoint] = TxState::BothFreeOddFirst,
                            _ => if (bdt as *mut Bdt as usize).get_bit(3) {
                                    state.tx_state[endpoint] = TxState::OddFree
                                } else {
                                    state.tx_state[endpoint] = TxState::EvenFree
                                },
                        }
                    }
                } else { // receive
                    let packet = packet.as_mut();
                    packet.len = bdt.byte_count();
                    if packet.len > 0 {
                        packet.index = 0;
                        packet.next = None;
                    } else {
                        let data = if (bdt as *mut Bdt as u32).get_bit(3) {
                            Data::Data1
                        } else {
                            Data::Data0
                        };
                        bdt.set_desc(64, data);
                    }
                }
            }
            reg.istat.write(InterruptFlags::TOKEN_DONE.bits());
            continue; // back to the top
        }

        if status.contains(InterruptFlags::USB_RESET) {
            // initialize BDT toggle bits
            reg.ctl.write(0x02);
            state.ep0_tx_bdt_bank = Pair::Even;

            let mut tables = USB0_BDTS.borrow(cs).borrow_mut();
            // set up buffers to receive Setup and OUT packets
            {
                let mut bdt = tables[(0, Direction::Rx, Pair::Even)];
                bdt.set_desc(EP0_SIZE, Data::Data0);
                bdt.addr = state.ep0_rx0_buf.as_mut_ptr() as *mut u8;
            }
            {
                let mut bdt = tables[(0, Direction::Rx, Pair::Odd)];
                bdt.set_desc(EP0_SIZE, Data::Data0);
                bdt.addr = state.ep0_rx1_buf.as_mut_ptr() as *mut u8;
            }
            tables[(0, Direction::Tx, Pair::Even)].desc = 0;
            tables[(0, Direction::Tx, Pair::Odd)].desc = 0;

            // activate endpoint 0
            // TODO: replace with a bitflag struct
            let mut endpt0 = 0;
            endpt0.set_bit(3, true); // RX enable
            endpt0.set_bit(2, true); // TX enable
            endpt0.set_bit(0, true); // Handshaking enable
            reg.endpt0.write(endpt0);

            // clear all ending interrupts
            reg.errstat.write(0xFF);
            reg.istat.write(0xFF);

            // set the address to zero during enumeration
            reg.addr.write(0);

            // enable other interrupts
            reg.erren.write(0xFF);
            reg.inten.write((
                InterruptFlags::TOKEN_DONE |
                InterruptFlags::SOF_THRESHOLD_OK |
                InterruptFlags::STALL |
                InterruptFlags::ERROR |
                InterruptFlags::USB_RESET |
                InterruptFlags::SLEEP
            ).bits());

            // is this necessary?
            reg.ctl.write(0x01); // USBENSOFEN
            return
        }

        if status.contains(InterruptFlags::STALL) {
            // TODO: replace with a bitflag struct
            reg.endpt0.modify(|mut endpt0| {
                endpt0.set_bit(1, false); // clear stall
                endpt0
            });
            reg.istat.write(InterruptFlags::STALL.bits());
        }

        if status.contains(InterruptFlags::ERROR) {
            let err = reg.errstat.read();
            reg.errstat.write(err);
            reg.istat.write(InterruptFlags::ERROR.bits());
        }

        if status.contains(InterruptFlags::SLEEP) {
            reg.istat.write(InterruptFlags::SLEEP.bits());
        }

        break;
    }
}

#[derive(Clone, Copy, PartialEq)]
enum StandardRequestCode {
    GetStatus       = 0x00,
    ClearFeature    = 0x01,
    SetFeature      = 0x03,
    SetAddress      = 0x05,
    GetDescriptor   = 0x06,
    SetDescriptor   = 0x07,
    GetConfiguration = 0x08,
    SetConfiguration = 0x09,
    GetInterface    = 0x0A,
    SetInterface    = 0x0B,
    SyncFrame       = 0x0C,
}

impl From<u8> for StandardRequestCode {
    fn from(val: u8) -> StandardRequestCode {
        match val {
            0 => StandardRequestCode::GetStatus,
            1 => StandardRequestCode::ClearFeature,
            3 => StandardRequestCode::SetFeature,
            5 => StandardRequestCode::SetAddress,
            6 => StandardRequestCode::GetDescriptor,
            7 => StandardRequestCode::SetDescriptor,
            8 => StandardRequestCode::GetConfiguration,
            9 => StandardRequestCode::SetConfiguration,
            10 => StandardRequestCode::GetInterface,
            11 => StandardRequestCode::SetInterface,
            12 => StandardRequestCode::SyncFrame,
            _ => unreachable!(),
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum CdcRequestCode {
    SendEncapsulatedCommand = 0x00,
    GetEncapsulatedResponse = 0x01,
    SetCommFeature          = 0x02,
    GetCommFeature          = 0x03,
    ClearCommFeature        = 0x04,

    SetAuxLineState = 0x10,
    SetHookState    = 0x11,
    PulseSetup      = 0x12,
    SendPulse       = 0x13,
    SetPulseTime    = 0x14,
    RingAuxJack     = 0x15,

    SetLineCoding       = 0x20,
    GetLineCoding       = 0x21,
    SetControlLineState = 0x22,
    SendBreak           = 0x23,

    SetRingerParams     = 0x30,
    GetRingerParams     = 0x31,
    SetOperationParams  = 0x32,
    GetOperationParams  = 0x33,
}

impl From<u8> for CdcRequestCode {
    fn from(val: u8) -> CdcRequestCode {
        match val {
            0x00 => CdcRequestCode::SendEncapsulatedCommand,
            0x01 => CdcRequestCode::GetEncapsulatedResponse,
            0x02 => CdcRequestCode::SetCommFeature,
            0x03 => CdcRequestCode::GetCommFeature,
            0x04 => CdcRequestCode::ClearCommFeature,

            0x10 => CdcRequestCode::SetAuxLineState,
            0x11 => CdcRequestCode::SetHookState,
            0x12 => CdcRequestCode::PulseSetup,
            0x13 => CdcRequestCode::SendPulse,
            0x14 => CdcRequestCode::SetPulseTime,
            0x15 => CdcRequestCode::RingAuxJack,

            0x20 => CdcRequestCode::SetLineCoding,
            0x21 => CdcRequestCode::GetLineCoding,
            0x22 => CdcRequestCode::SetControlLineState,
            0x23 => CdcRequestCode::SendBreak,

            0x30 => CdcRequestCode::SetRingerParams,
            0x31 => CdcRequestCode::GetRingerParams,
            0x32 => CdcRequestCode::SetOperationParams,
            0x33 => CdcRequestCode::GetOperationParams,

            _ => unreachable!(),
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum HidRequestCode {
    GetReport   = 0x01,
    GetIdle     = 0x02,
    GetProtocol = 0x03,
    SetReport   = 0x09,
    SetIdle     = 0x0A,
    SetProtocol = 0x0B,
}

impl From<u8> for HidRequestCode {
    fn from(val: u8) -> HidRequestCode {
        match val {
            0x01 => HidRequestCode::GetReport,
            0x02 => HidRequestCode::GetIdle,
            0x03 => HidRequestCode::GetProtocol,
            0x09 => HidRequestCode::SetReport,
            0x0A => HidRequestCode::SetIdle,
            0x0B => HidRequestCode::SetProtocol,

            _ => unreachable!(),
        }
    }
}

#[derive(Clone, Copy)]
union ClassRequest {
    cdc: CdcRequestCode,
    hid: HidRequestCode,
    raw: u8,
}

#[derive(Clone, Copy)]
enum RequestType {
    Standard(StandardRequestCode),
    Class(ClassRequest),
    Vendor(u8),
    Reserved(u8),
}

#[derive(Clone, Copy, PartialEq)]
enum RequestDirection {
    /// Host to device
    Out = 0,
    /// Device to host
    In = 1,
}

#[derive(Clone, Copy, PartialEq)]
enum Recipient {
    Device = 0,
    Interface = 1,
    Endpoint = 2,
    Other = 3,
}

/// [7]: Request direction
///     0 = Host to device - Out
///     1 = Device to host - In
/// [5-6]: Request type
///     0 = standard
///     1 = class
///     2 = vendor
///     3 = reserved
/// [0-4]: Recipient
///     0 = device
///     1 = interface
///     2 = endpoint
///     3 = other
fn parse_request_type(request_ty: u8, request: u8) -> (RequestDirection, RequestType, Recipient) {
    let dir = if request_ty.get_bit(7) { RequestDirection::In } else { RequestDirection::Out };
    let ty = match request_ty.get_bits(5..7) {
        0 => RequestType::Standard(request.into()),
        1 => RequestType::Class(ClassRequest { raw: request }),
        2 => RequestType::Vendor(request),
        3 => RequestType::Reserved(request),
        _ => unreachable!(),
    };
    let recipient = match request_ty.get_bits(0..5) {
        0 => Recipient::Device,
        1 => Recipient::Interface,
        2 => Recipient::Endpoint,
        3 => Recipient::Other,
        _ => unreachable!(),
    };
    (dir, ty, recipient)
}

struct Setup {
    request_dir: RequestDirection,
    request_type: RequestType,
    recipient: Recipient,
    value: u16,
    index: u16,
    len: u16,
}

impl Setup {
    const fn new() -> Self {
        Setup {
            request_dir: RequestDirection::Out,
            request_type: RequestType::Standard(StandardRequestCode::GetStatus),
            recipient: Recipient::Device,
            value: 0,
            index: 0,
            len: 0,
        }
    }
}

static mut SETUP: Mutex<RefCell<Setup>> = Mutex::new(RefCell::new(Setup::new()));

unsafe fn usb_control(cs: &CriticalSection, reg: &mut UsbRegs, state: &mut IsrState, stat: u8) {
    let tables = &mut *USB0_BDTS.borrow(cs).borrow_mut();
    let setup = &mut *SETUP.borrow(cs).borrow_mut();
    let pid = {
        let bdt = tables.stat_to_bdt(stat);
        bdt.token_pid()
    };
    

    match pid {
        TokenPid::Setup => {
            {
                let bdt = tables.stat_to_bdt(stat);
                if bdt.byte_count() != 8 {
                    // error
                }

                let buf = slice::from_raw_parts_mut(bdt.addr, 8);

                let (request_dir, request_type, recipient) = parse_request_type(buf[0], buf[1]);
                setup.request_dir = request_dir;
                setup.request_type = request_type;
                setup.recipient = recipient;
                setup.value = ((buf[2] as u16) << 8) | buf[3] as u16;
                setup.index = ((buf[4] as u16) << 8) | buf[5] as u16;
                setup.len = ((buf[6] as u16) << 8) | buf[7] as u16;

                // give buffer back
                bdt.set_desc(EP0_SIZE, Data::Data1);
            }

            // clear any leftover pending IN transactions
            state.ep0_tx_ptr = None;
            tables[(0, Direction::Tx, Pair::Even)].desc = 0;
            tables[(0, Direction::Tx, Pair::Odd)].desc = 0;
            // first IN after Setup is always DATA1
            state.ep0_tx_data_toggle = Data::Data1;

            usb_setup(cs, reg, state, setup, tables);
        }
        TokenPid::Ack | TokenPid::Out => {
            // request_type = 0x21 - Out, class, interface
            // request = 0x20 - SetLineCoding
            {
                if let RequestType::Class(ClassRequest { cdc: CdcRequestCode::SetLineCoding }) = setup.request_type {
                    if let Some(ref mut cdc) = *CDC_DATA_INTERFACE.borrow(cs).borrow_mut() {
                        {
                            let bdt = tables.stat_to_bdt(stat);
                            let buf = slice::from_raw_parts_mut(bdt.addr as *mut u32, 2);
                            cdc.line_coding[0] = buf[0];
                            cdc.line_coding[1] = buf[1];
                        }
                        // Teensy USB reboot command
                        if cdc.line_coding[0] == 134 {
                            state.reboot_timer = 15;
                        }
                        endpoint0_transmit(cs, state, tables, null_mut(), 0);
                    }
                }
            }
            // request_type = 0x21 - Out, class, interface
            // request = 0x09 - SetReport
            // value = 0x0200 - Output, no report ID
            // index = KEYBOARD_INTERFACE
            // length = 0x0001
            /*
            TODO:
            {
                if (setup.word1 == 0x0200_09_21 && setup.word2 == ((1<<16)|KEYBOARD_INTERFACE)) {
                    keyboard_leds = buf[0];
                    endpoint0_transmit(NULL, 0);
                }
            }
            */
            // Teensy USB reboot command
            // request_type = 0x21 - Out, class, interface
            // request = 0x09 - SetReport
            // value = 0x0300 - Feature, no report ID
            // index = SEREMU_INTERFACE
            // length = 0x0004
            #[cfg(feature = "usb-seremu")]
            {
                let buf = slice::from_raw_parts_mut(bdt.addr, 4);
                if let (
                    RequestDirection::Out,
                    RequestType::Class(ClassRequest { hid: HidRequestCode::SetReport }),
                    Recipient::Interface,
                    0x03000,
                    0x0004,
                    0xA9, 0x45, 0xC2, 0x6B,
                ) = (
                    setup.request_dir,
                    setup.request_type,
                    setup.recipient,
                    setup.value,
                    setup.len,
                    buf[0], buf[1], buf[2], buf[3],
                ) {
                    if setup.index == state.seremu_interface.into() {
                        state.reboot_timer = 5;
                        endpoint0_transmit(None);
                    }
                }
            }
            /*
            TODO:
            if (usb_audio_set_feature(&setup, buf)) {
                endpoint0_transmit(NULL, 0);
            }
            */

            // give buffer back
            let bdt = tables.stat_to_bdt(stat);
            bdt.set_desc(EP0_SIZE, Data::Data1);
        }
        TokenPid::In => {
            // send remaining data if any
            let data = state.ep0_tx_ptr;
            if let Some((data, len)) = data {
                // TODO: used in setup too, refactor for code reuse
                let size = if len > EP0_SIZE {
                    EP0_SIZE
                } else {
                    len
                };
                endpoint0_transmit(cs, state, tables, data.as_ptr(), size);
                state.ep0_tx_ptr = if len - size > 0 {
                    Some((data, len - size))
                } else {
                    None
                };
            }

            if let RequestType::Standard(StandardRequestCode::SetAddress) = setup.request_type {
                setup.request_type = RequestType::Standard(StandardRequestCode::GetStatus);
                reg.addr.write(setup.value as u8);
            }
        }
        _ => unreachable!(),
    }

    // clear TXSUSPENDTOKENBUSY bit
    reg.ctl.modify(|mut ctl| {
        ctl.set_bit(5, false);
        ctl
    });
}

unsafe fn usb_setup(cs: &CriticalSection, reg: &mut UsbRegs, state: &mut IsrState, setup: &mut Setup, tables: &mut Tables) {
    // TODO: for interface commands, double check they are sending to a valid interface
    let (data, len) = match (setup.request_dir, setup.request_type, setup.recipient) {
        (RequestDirection::Out, RequestType::Standard(StandardRequestCode::SetAddress), Recipient::Device) => {
            // TODO: nop in teensy code
            (null_mut(), 0)
        }
        (RequestDirection::Out, RequestType::Standard(StandardRequestCode::SetConfiguration), Recipient::Device) => {
            unimplemented!()
        }
        (RequestDirection::In, RequestType::Standard(StandardRequestCode::GetConfiguration), Recipient::Device) => {
            state.reply_buffer[0] = state.configuration;
            (state.reply_buffer.as_mut_ptr(), 1)
        }
        (RequestDirection::In, RequestType::Standard(StandardRequestCode::GetStatus), Recipient::Device) => {
            // TODO: make these setable
            // bit 0: self powered yes/no
            // bit 1: remote wakeup enabled
            state.reply_buffer[0] = 0;
            state.reply_buffer[1] = 0;
            (state.reply_buffer.as_mut_ptr(), 1)
        }
        (RequestDirection::In, RequestType::Standard(StandardRequestCode::GetStatus), Recipient::Endpoint) => {
            unimplemented!()
        }
        (RequestDirection::Out, RequestType::Standard(StandardRequestCode::ClearFeature), Recipient::Endpoint) => {
            let endpoint = setup.index & 0x0F;
            if endpoint as usize > NUM_ENDPOINTS || setup.value != 0 {
                endpoint0_stall(reg);
                return
            }
            match endpoint {
                0 => &mut reg.endpt0,
                1 => &mut reg.endpt1,
                2 => &mut reg.endpt2,
                3 => &mut reg.endpt3,
                4 => &mut reg.endpt4,
                5 => &mut reg.endpt5,
                6 => &mut reg.endpt6,
                7 => &mut reg.endpt7,
                8 => &mut reg.endpt8,
                9 => &mut reg.endpt9,
                10 => &mut reg.endpt10,
                11 => &mut reg.endpt11,
                12 => &mut reg.endpt12,
                13 => &mut reg.endpt13,
                14 => &mut reg.endpt14,
                15 => &mut reg.endpt15,
                _ => unreachable!(),
            }.modify(|mut endpt| {
                endpt.set_bit(1, false);
                endpt
            });
            (null_mut(), 0)
        }
        (RequestDirection::Out, RequestType::Standard(StandardRequestCode::SetFeature), Recipient::Endpoint) => {
            let endpoint = setup.index & 0x0F;
            if endpoint as usize > NUM_ENDPOINTS || setup.value != 0 {
                endpoint0_stall(reg);
                return
            }
            match endpoint {
                0 => &mut reg.endpt0,
                1 => &mut reg.endpt1,
                2 => &mut reg.endpt2,
                3 => &mut reg.endpt3,
                4 => &mut reg.endpt4,
                5 => &mut reg.endpt5,
                6 => &mut reg.endpt6,
                7 => &mut reg.endpt7,
                8 => &mut reg.endpt8,
                9 => &mut reg.endpt9,
                10 => &mut reg.endpt10,
                11 => &mut reg.endpt11,
                12 => &mut reg.endpt12,
                13 => &mut reg.endpt13,
                14 => &mut reg.endpt14,
                15 => &mut reg.endpt15,
                _ => unreachable!(),
            }.modify(|mut endpt| {
                endpt.set_bit(1, true);
                endpt
            });
            // TODO: clear data toggle?
            (null_mut(), 0)
        }
        (RequestDirection::In, RequestType::Standard(StandardRequestCode::GetDescriptor), Recipient::Device) |
        (RequestDirection::In, RequestType::Standard(StandardRequestCode::GetDescriptor), Recipient::Interface) => {
            unimplemented!()
        }
        (RequestDirection::Out, RequestType::Class(ClassRequest { cdc: CdcRequestCode::SetControlLineState }), Recipient::Interface) => {
            if let Some(ref mut cdc) = *CDC_DATA_INTERFACE.borrow(cs).borrow_mut() {
                // TODO: enum
                // bit 0: DTR
                // bit 1: RTS
                cdc.line_rtsdtr = setup.value as u8;
                (null_mut(), 0)
            } else {
                endpoint0_stall(reg);
                return
            }
        }
        (RequestDirection::Out, RequestType::Class(ClassRequest { cdc: CdcRequestCode::SendBreak }), Recipient::Interface) => {
            if (*CDC_DATA_INTERFACE.borrow(cs).borrow()).is_some() {
                // TODO: nop in teensy code
                (null_mut(), 0)
            } else {
                endpoint0_stall(reg);
                return
            }
        }
        (RequestDirection::Out, RequestType::Class(ClassRequest { cdc: CdcRequestCode::SetLineCoding }), Recipient::Interface) => {
            if (*CDC_DATA_INTERFACE.borrow(cs).borrow()).is_some() {
                return
            } else {
                endpoint0_stall(reg);
                return
            }
        }
        _ => {
            endpoint0_stall(reg);
            return
        }
    };

    // TODO:
}

fn endpoint0_transmit(cs: &CriticalSection, state: &mut IsrState, tables: &mut Tables, data: *mut u8, len: usize) {
    let mut bdt = tables[(0, Direction::Tx, state.ep0_tx_bdt_bank)];
    bdt.addr = data;
    bdt.set_desc(len, state.ep0_tx_data_toggle);
    state.ep0_tx_data_toggle = if state.ep0_tx_data_toggle == Data::Data0 { Data::Data1 } else { Data::Data0 };
    state.ep0_tx_bdt_bank = if state.ep0_tx_bdt_bank == Pair::Even { Pair::Odd } else { Pair::Even };
}

unsafe fn endpoint0_stall(reg: &mut UsbRegs) {
    reg.endpt0.modify(|mut endpt0| {
        endpt0.set_bit(1, true); // set stall
        endpt0
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn usb0_regs() {
        unsafe {
            let reg = & *(USB0_ADDR as *const UsbRegs);
            assert_eq!(0x4007_2000 as *const UsbRO, &reg.perid      as *const UsbRO, "perid");
            assert_eq!(0x4007_2004 as *const UsbRO, &reg.idcomp     as *const UsbRO, "idcomp");
            assert_eq!(0x4007_2008 as *const UsbRO, &reg.rev        as *const UsbRO, "rev");
            assert_eq!(0x4007_200C as *const UsbRO, &reg.addinfo    as *const UsbRO, "addinfo");
            assert_eq!(0x4007_2010 as *const UsbRW, &reg.otgistat   as *const UsbRW, "otgistat");
            assert_eq!(0x4007_2014 as *const UsbRW, &reg.otgicr     as *const UsbRW, "otgicr");
            assert_eq!(0x4007_2018 as *const UsbRW, &reg.otgstat    as *const UsbRW, "otgstat");
            assert_eq!(0x4007_201C as *const UsbRW, &reg.otgctl     as *const UsbRW, "otgctl");
            assert_eq!(0x4007_2080 as *const UsbRW, &reg.istat      as *const UsbRW, "istat");
            assert_eq!(0x4007_2084 as *const UsbRW, &reg.inten      as *const UsbRW, "inten");
            assert_eq!(0x4007_2088 as *const UsbRW, &reg.errstat    as *const UsbRW, "errstat");
            assert_eq!(0x4007_208C as *const UsbRW, &reg.erren      as *const UsbRW, "erren");
            assert_eq!(0x4007_2090 as *const UsbRO, &reg.stat       as *const UsbRO, "stat");
            assert_eq!(0x4007_2094 as *const UsbRW, &reg.ctl        as *const UsbRW, "ctl");
            assert_eq!(0x4007_2098 as *const UsbRW, &reg.addr       as *const UsbRW, "addr");
            assert_eq!(0x4007_209C as *const UsbRW, &reg.bdtpage1   as *const UsbRW, "bdtpage1");
            assert_eq!(0x4007_20A0 as *const UsbRW, &reg.frmnuml    as *const UsbRW, "frmnuml");
            assert_eq!(0x4007_20A4 as *const UsbRW, &reg.frmnumh    as *const UsbRW, "frmnumh");
            assert_eq!(0x4007_20A8 as *const UsbRW, &reg.token      as *const UsbRW, "token");
            assert_eq!(0x4007_20AC as *const UsbRW, &reg.softhld    as *const UsbRW, "softhld");
            assert_eq!(0x4007_20B0 as *const UsbRW, &reg.bdtpage2   as *const UsbRW, "bdtpage2");
            assert_eq!(0x4007_20B4 as *const UsbRW, &reg.bdtpage3   as *const UsbRW, "bdtpage3");
            assert_eq!(0x4007_20C0 as *const UsbRW, &reg.endpt0     as *const UsbRW, "endpt0");
            assert_eq!(0x4007_20C4 as *const UsbRW, &reg.endpt1     as *const UsbRW, "endpt1");
            assert_eq!(0x4007_20C8 as *const UsbRW, &reg.endpt2     as *const UsbRW, "endpt2");
            assert_eq!(0x4007_20CC as *const UsbRW, &reg.endpt3     as *const UsbRW, "endpt3");
            assert_eq!(0x4007_20D0 as *const UsbRW, &reg.endpt4     as *const UsbRW, "endpt4");
            assert_eq!(0x4007_20D4 as *const UsbRW, &reg.endpt5     as *const UsbRW, "endpt5");
            assert_eq!(0x4007_20D8 as *const UsbRW, &reg.endpt6     as *const UsbRW, "endpt6");
            assert_eq!(0x4007_20DC as *const UsbRW, &reg.endpt7     as *const UsbRW, "endpt7");
            assert_eq!(0x4007_20E0 as *const UsbRW, &reg.endpt8     as *const UsbRW, "endpt8");
            assert_eq!(0x4007_20E4 as *const UsbRW, &reg.endpt9     as *const UsbRW, "endpt9");
            assert_eq!(0x4007_20E8 as *const UsbRW, &reg.endpt10    as *const UsbRW, "endpt10");
            assert_eq!(0x4007_20EC as *const UsbRW, &reg.endpt11    as *const UsbRW, "endpt11");
            assert_eq!(0x4007_20F0 as *const UsbRW, &reg.endpt12    as *const UsbRW, "endpt12");
            assert_eq!(0x4007_20F4 as *const UsbRW, &reg.endpt13    as *const UsbRW, "endpt13");
            assert_eq!(0x4007_20F8 as *const UsbRW, &reg.endpt14    as *const UsbRW, "endpt14");
            assert_eq!(0x4007_20FC as *const UsbRW, &reg.endpt15    as *const UsbRW, "endpt15");
            assert_eq!(0x4007_2100 as *const UsbRW, &reg.usbctrl    as *const UsbRW, "usbctrl");
            assert_eq!(0x4007_2104 as *const UsbRO, &reg.observe    as *const UsbRO, "observe");
            assert_eq!(0x4007_2108 as *const UsbRW, &reg.control    as *const UsbRW, "control");
            assert_eq!(0x4007_210C as *const UsbRW, &reg.usbtrc0    as *const UsbRW, "usbtrc0");
            assert_eq!(0x4007_2114 as *const UsbRW, &reg.usbfrmadjust as *const UsbRW, "usbfrmadjust");
        }
    }

    #[test]
    fn tables_alignment_size() {
        use core::mem;

        // Double check the alignment of the BDTs doesn't shift.
        // Don't want it going evil on us.
        assert_eq!(mem::align_of::<Tables>(), 512);
        // Double check the size as well in case the Mutex adds size or it eats too much pie.
        assert_eq!(mem::size_of::<Tables>(), 512);
    }
}
