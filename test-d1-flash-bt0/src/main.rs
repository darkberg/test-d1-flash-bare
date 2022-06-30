#![feature(naked_functions, asm_sym, asm_const)]
#![feature(default_alloc_error_handler)]
#![feature(let_chains)]
#![feature(once_cell)]
#![no_std]
#![no_main]

use core::intrinsics::transmute;
use core::ptr::{read_volatile, write_volatile};
use core::{arch::asm, panic::PanicInfo};
use d1_pac::Peripherals;
use embedded_hal::digital::{blocking::OutputPin, PinState};
//use std::io;

#[macro_use]
mod logging;
mod ccu;
mod gpio;
mod jtag;
mod mctl;
mod spi;
mod spi_flash;
mod time;
mod uart;

use ccu::Clocks;
use gpio::Gpio;
use jtag::Jtag;
use mctl::RAM_BASE;
use spi::Spi;
use spi_flash::{SpiNand, SpiNor};
use time::U32Ext;
use uart::{Config, Parity, Serial, StopBits, WordLength};

// taken from oreboot
pub type EntryPoint = unsafe extern "C" fn(r0: usize, r1: usize);

const STACK_SIZE: usize = 1 * 1024; // 1KiB

const GPIO_BASE_ADDR: u32 = 0x0200_0000;
const PWM_BASE_ADDR: u32 = 0x0200_0C00; //PWM Base Address

const GPIO_PB_CFG0: u32 = GPIO_BASE_ADDR + 0x0030;//PB Configure Reg 0
const GPIO_PB_CFG1: u32 = GPIO_BASE_ADDR + 0x0034;//PB Configure Reg 1
const GPIO_PB_DATA:  u32 = GPIO_BASE_ADDR + 0x0040;//PB Data Reg 
const GPIO_PB_DRV0: u32 = GPIO_BASE_ADDR + 0x0044;//PB Multi Driving Reg 0
const GPIO_PB_DRV1: u32 = GPIO_BASE_ADDR + 0x0048;//PB Multi Driving Reg 1
const GPIO_PB_PULL0:u32 = GPIO_BASE_ADDR + 0x0054;//PB Pull Reg 0

const GPIO_PC_CFG0: u32 = GPIO_BASE_ADDR + 0x0060;
const GPIO_PC_DATA: u32 = GPIO_BASE_ADDR + 0x0070;

const GPIO_PB_EINT_CFG0: u32 = GPIO_BASE_ADDR + 0x0220;//PB External Interrupt Configure Register 0

const GPIO_PB_EINT_CTL:    u32 = GPIO_BASE_ADDR + 0x0230;//PB External Interrupt Control Register
const GPIO_PB_EINT_STATUS: u32 = GPIO_BASE_ADDR + 0x0234;//PB External Interrupt Status Register
const GPIO_PB_EINT_DEB:    u32 = GPIO_BASE_ADDR + 0x0238;//PB External Interrupt Debounce Register

#[link_section = ".bss.uninit"]
static mut SBI_STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];


//PWM Registers list
const PWM_PIER_ADDR: u32=PWM_BASE_ADDR + 0x0000;//PWM IRQ Enable Register
const PWM_PISR_ADDR: u32=PWM_BASE_ADDR + 0x0004;//PWM IRQ Status Register
const PWM_CIER_ADDR: u32=PWM_BASE_ADDR + 0x0010;//Capture IRQ Enable Register
const PWM_CISR_ADDR: u32=PWM_BASE_ADDR + 0x0014;//Capture IRQ Status Register
const PWM_PCCR01_ADDR: u32=PWM_BASE_ADDR + 0x0020;//PWM01 Clock Configuration Register
const PWM_PCCR23_ADDR: u32=PWM_BASE_ADDR + 0x0024;//PWM23 Clock Configuration Register
const PWM_PCCR45_ADDR: u32=PWM_BASE_ADDR + 0x0028;//PWM45 Clock Configuration Register
const PWM_PCCR67_ADDR: u32=PWM_BASE_ADDR + 0x002C;//PWM67 Clock Configuration Register

const PWM_PCGR_ADDR: u32=PWM_BASE_ADDR + 0x0040;//PWM Clock Gating Register
const PWM_PDZCR01_ADDR: u32=PWM_BASE_ADDR + 0x0060;//PWM01 Dead Zone Control Register
const PWM_PDZCR23_ADDR: u32=PWM_BASE_ADDR + 0x0064;//PWM23 Dead Zone Control Register
const PWM_PDZCR45_ADDR: u32=PWM_BASE_ADDR + 0x0068;//PWM45 Dead Zone Control Register
const PWM_PDZCR67_ADDR: u32=PWM_BASE_ADDR + 0x006C;//PWM67 Dead Zone Control Register

const PWM_PER_ADDR: u32=PWM_BASE_ADDR + 0x0080;//PWM Enable Register
const PWM_PGR0_ADDR: u32=PWM_BASE_ADDR + 0x0090;//PWM Group0 Register
const PWM_PGR1_ADDR: u32=PWM_BASE_ADDR + 0x0094;//PWM Group1 Register
const PWM_PGR2_ADDR: u32=PWM_BASE_ADDR + 0x0098;//PWM Group2 Register
const PWM_PGR3_ADDR: u32=PWM_BASE_ADDR + 0x009C;//PWM Group3 Register


const PWM_CER_ADDR: u32=PWM_BASE_ADDR + 0x00C0;//Capture Enable Register

const PWM_PCR0_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0000+0*0x0020;//PWM0 Control Register
const PWM_PCR1_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0000+1*0x0020;//PWM1 Control Register
const PWM_PCR2_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0000+2*0x0020;//PWM2 Control Register
const PWM_PCR3_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0000+3*0x0020;//PWM3 Control Register
const PWM_PCR4_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0000+4*0x0020;//PWM4 Control Register
const PWM_PCR5_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0000+5*0x0020;//PWM5 Control Register
const PWM_PCR6_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0000+6*0x0020;//PWM6 Control Register
const PWM_PCR7_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0000+7*0x0020;//PWM7 Control Register

const PWM_PPR0_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0004+0*0x0020;//PWM0 Period Register
const PWM_PPR1_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0004+1*0x0020;//PWM1 Period Register
const PWM_PPR2_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0004+2*0x0020;//PWM2 Period Register
const PWM_PPR3_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0004+3*0x0020;//PWM3 Period Register
const PWM_PPR4_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0004+4*0x0020;//PWM4 Period Register
const PWM_PPR5_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0004+5*0x0020;//PWM5 Period Register
const PWM_PPR6_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0004+6*0x0020;//PWM6 Period Register
const PWM_PPR7_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0004+7*0x0020;//PWM7 Period Register

const PWM_PCNTR0_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0008+0*0x0020;//PWM0 Count Register
const PWM_PCNTR1_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0008+1*0x0020;//PWM1 Count Register
const PWM_PCNTR2_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0008+2*0x0020;//PWM2 Count Register
const PWM_PCNTR3_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0008+3*0x0020;//PWM3 Count Register
const PWM_PCNTR4_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0008+4*0x0020;//PWM4 Count Register
const PWM_PCNTR5_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0008+5*0x0020;//PWM5 Count Register
const PWM_PCNTR6_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0008+6*0x0020;//PWM6 Count Register
const PWM_PCNTR7_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0008+7*0x0020;//PWM7 Count Register


const PWM_PPCNTR0_ADDR: u32=PWM_BASE_ADDR +0x0100+0x000C+0*0x0020;//PWM0 Pulse Count Register
const PWM_PPCNTR1_ADDR: u32=PWM_BASE_ADDR +0x0100+0x000C+1*0x0020;//PWM1 Pulse Count Register
const PWM_PPCNTR2_ADDR: u32=PWM_BASE_ADDR +0x0100+0x000C+2*0x0020;//PWM2 Pulse Count Register
const PWM_PPCNTR3_ADDR: u32=PWM_BASE_ADDR +0x0100+0x000C+3*0x0020;//PWM3 Pulse Count Register
const PWM_PPCNTR4_ADDR: u32=PWM_BASE_ADDR +0x0100+0x000C+4*0x0020;//PWM4 Pulse Count Register
const PWM_PPCNTR5_ADDR: u32=PWM_BASE_ADDR +0x0100+0x000C+5*0x0020;//PWM5 Pulse Count Register
const PWM_PPCNTR6_ADDR: u32=PWM_BASE_ADDR +0x0100+0x000C+6*0x0020;//PWM6 Pulse Count Register
const PWM_PPCNTR7_ADDR: u32=PWM_BASE_ADDR +0x0100+0x000C+7*0x0020;//PWM7 Pulse Count Register


const PWM_CCR0_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0010+0*0x0020;//Capture0 Control Register
const PWM_CCR1_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0010+1*0x0020;//Capture1 Control Register
const PWM_CCR2_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0010+2*0x0020;//Capture2 Control Register
const PWM_CCR3_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0010+3*0x0020;//Capture3 Control Register
const PWM_CCR4_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0010+4*0x0020;//Capture4 Control Register
const PWM_CCR5_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0010+5*0x0020;//Capture5 Control Register
const PWM_CCR6_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0010+6*0x0020;//Capture6 Control Register
const PWM_CCR7_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0010+7*0x0020;//Capture7 Control Register

const PWM_CRLR0_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0014+0*0x0020;//Capture0 Rise Lock Register
const PWM_CRLR1_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0014+1*0x0020;//Capture1 Rise Lock Register
const PWM_CRLR2_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0014+2*0x0020;//Capture2 Rise Lock Register
const PWM_CRLR3_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0014+3*0x0020;//Capture3 Rise Lock Register
const PWM_CRLR4_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0014+4*0x0020;//Capture4 Rise Lock Register
const PWM_CRLR5_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0014+5*0x0020;//Capture5 Rise Lock Register
const PWM_CRLR6_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0014+6*0x0020;//Capture6 Rise Lock Register
const PWM_CRLR7_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0014+7*0x0020;//Capture7 Rise Lock Register

const PWM_CFLR0_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0018+0*0x0020;//Capture0 Fall Lock Register
const PWM_CFLR1_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0018+1*0x0020;//Capture1 Fall Lock Register
const PWM_CFLR2_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0018+2*0x0020;//Capture2 Fall Lock Register
const PWM_CFLR3_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0018+3*0x0020;//Capture3 Fall Lock Register
const PWM_CFLR4_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0018+4*0x0020;//Capture4 Fall Lock Register
const PWM_CFLR5_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0018+5*0x0020;//Capture5 Fall Lock Register
const PWM_CFLR6_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0018+6*0x0020;//Capture6 Fall Lock Register
const PWM_CFLR7_ADDR: u32=PWM_BASE_ADDR +0x0100+0x0018+7*0x0020;//Capture7 Fall Lock Register




//End Pwm registers list

/// Jump over head data to executable code.
///
/// # Safety
///
/// Naked function.
#[naked]
#[link_section = ".head.text"]
#[export_name = "head_jump"]
pub unsafe extern "C" fn head_jump() {
    asm!(
        ".option push",
        ".option rvc",
        "c.j    0x60",
        ".option pop",
        // sym start,
        options(noreturn)
    )
}

// todo: option(noreturn) generates an extra `unimp` insn

#[repr(C)]
pub struct HeadData {
    magic: [u8; 8],
    checksum: u32,
    length: u32,
    pub_head_size: u32,
    fel_script_address: u32,
    fel_uenv_length: u32,
    dt_name_offset: u32,
    dram_size: u32,
    boot_media: u32,
    string_pool: [u32; 13],
}

const STAMP_CHECKSUM: u32 = 0x5F0A6C39;

// clobber used by KEEP(*(.head.data)) in link script
#[link_section = ".head.data"]
pub static HEAD_DATA: HeadData = HeadData {
    magic: *b"eGON.BT0",
    checksum: STAMP_CHECKSUM, // real checksum filled by blob generator
    length: 0,                // real size filled by blob generator
    pub_head_size: 0,
    fel_script_address: 0,
    fel_uenv_length: 0,
    dt_name_offset: 0,
    dram_size: 0,
    boot_media: 0,
    string_pool: [0; 13],
};

/// Jump over head data to executable code.
///
/// # Safety
///
/// Naked function.
#[naked]
#[export_name = "start"]
#[link_section = ".text.entry"]
pub unsafe extern "C" fn start() -> ! {
    asm!(
        // 1. clear cache and processor states
        "csrw   mie, zero",
        "li     t2, 0x30013",
        "csrs   0x7c2, t2", // MCOR
        // 2. initialize programming langauge runtime
        // clear bss segment
        "la     t0, sbss",
        "la     t1, ebss",
        "1:",
        "bgeu   t0, t1, 1f",
        "sd     x0, 0(t0)",
        "addi   t0, t0, 4",
        "j      1b",
        "1:",
        // does not init data segment as BT0 runs in sram
        // 3. prepare stack
        "la     sp, {stack}",
        "li     t0, {stack_size}",
        "add    sp, sp, t0",
        "la     a0, {head_data}",
        "j      {main}",
        "j      {cleanup}",
        stack      =   sym SBI_STACK,
        stack_size = const STACK_SIZE,
        head_data  =   sym HEAD_DATA,
        main       =   sym main,
        cleanup    =   sym cleanup,
        options(noreturn)
    )
}

extern "C" fn main() {
    // there was configure_ccu_clocks, but ROM code have already done configuring for us
    let p = Peripherals::take().unwrap();
    let clocks = Clocks {
        uart_clock: 24_000_000.hz(), // hard coded
    };
    let gpio = Gpio::new(p.GPIO);

    // configure jtag interface
    let tms = gpio.portf.pf0.into_function_4();
    let tck = gpio.portf.pf5.into_function_4();
    let tdi = gpio.portf.pf1.into_function_4();
    let tdo = gpio.portf.pf3.into_function_4();
    let _jtag = Jtag::new((tms, tck, tdi, tdo));

    // light up led
    let mut pb5 = gpio.portb.pb5.into_output();
    pb5.set_high().unwrap();
    // FIXME: This is broken. It worked before. Breakage happened in commit:
    // fd7f6b8bc2eebb25f888ded040566e591f037e9a
    let mut pc1 = gpio.portc.pc1.into_output();
    pc1.set_high().unwrap();

    // Change into output mode
    let pc_cfg0 = unsafe { read_volatile(GPIO_PC_CFG0 as *const u32) };
    let mut val = pc_cfg0 & 0xffffff0f | 0b0001 << 4;
    unsafe { write_volatile(GPIO_PC_CFG0 as *mut u32, val) };
    // Set pin to HIGH
    let pc_dat0 = unsafe { read_volatile(GPIO_PC_DATA as *const u32) };
    val = pc_dat0 | 0b1 << 1;
    unsafe { write_volatile(GPIO_PC_DATA as *mut u32, val) };

    //enable PB1 & PB0 on external pins
    // light up led
    let mut pb0 = gpio.portb.pb0.into_output();
    pb0.set_high().unwrap();
    let mut pb1 = gpio.portb.pb1.into_output();

    pb1.set_high().unwrap();    

//first try for PortB , PB0 & PB1

    // Change into output mode pins from PortB
    let pb_cfg0 = unsafe { read_volatile(GPIO_PB_CFG0 as *const u32) };
    let mut val = pb_cfg0 & 0xffffff00 | 0x11;//0x11; //0x11 = pb0&1 as outputs // 0x22 = pb0&1 as pwm3 & pwm4
    unsafe { write_volatile(GPIO_PB_CFG0 as *mut u32, val) };
    // Set pins to HIGH
    let pb_dat0 = unsafe { read_volatile(GPIO_PB_DATA as *const u32) };
    val = pb_dat0 | 0b11;
    unsafe { write_volatile(GPIO_PB_DATA as *mut u32, val) };

    //0x0238 PB External Interrupt Debounce Register (Default Value: 0x0000_0000)
    //GPIO_PB_EINT_DEB
    let mut pb_EINT_DEB0 = unsafe { read_volatile(GPIO_PB_EINT_DEB as *const u32) };
    //println!("M mmmm");//, pb_EINT_DEB0).ok();


//The timer logic module of PWM consists of one 16-bit up-counter (PCNTR) and three 16-bit parameters
//(PWM_ENTIRE_CYCLE, PWM_ACT_CYCLE, PWM_COUNTER_START). The PWM_ENTIRE_CYCLE is used to
//

//END first try for PortB , PB0 & PB1
/////////////////////////////////////
/// 

    // prepare serial port logger
    let tx = gpio.portb.pb8.into_function_6();
    let rx = gpio.portb.pb9.into_function_6();
    let config = Config {
        baudrate: 115200.bps(),
        wordlength: WordLength::Eight,
        parity: Parity::None,
        stopbits: StopBits::One,
    };
    let serial = Serial::new(p.UART0, (tx, rx), config, &clocks);
    crate::logging::set_logger(serial);

    println!("oreboot 🦀 🕹️🕹️🕹️🕹️ 🔋 🏁🏁🏁🏁");

    let ram_size = mctl::init();
    println!("{}M 🐏", ram_size);

    // prepare spi interface
    let sck = gpio.portc.pc2.into_function_2();
    let scs = gpio.portc.pc3.into_function_2();
    let mosi = gpio.portc.pc4.into_function_2();
    let miso = gpio.portc.pc5.into_function_2();
    let spi = Spi::new(p.SPI0, (sck, scs, mosi, miso), &clocks);
  
    let mut flash = SpiNor::new(spi);

    // e.g., GigaDevice (GD) is 0xC8 and GD25Q128 is 0x4018
    // see flashrom/flashchips.h for details and more
    let id = flash.read_id();
    println!("SPI flash vendor 🏭 {:?} part 🍚 {:?} {:?}\n", id[0], id[1], id[2],);

    // 32K, the size of boot0
    let base = 0x1 << 15;
    let size: usize = 15400;
    for i in 0..size {
        let off = base + i * 4;
        let buf = flash.copy_into([(off >> 16) as u8, (off >> 8) as u8 % 255, off as u8 % 255]);

        let addr = RAM_BASE + i * 4;
        let val = u32::from_le_bytes([buf[3], buf[2], buf[1], buf[0]]);
        unsafe { write_volatile(addr as *mut u32, val) };
        let rval = unsafe { read_volatile(addr as *mut u32) };

        if rval != val {
            println!("MISMATCH  r 🦀");
            //{:08?} :: {:08?}", rval, val);
        }
        /*
        if i < 10 || i == 256 {
            println!("{:08x} :: {:08x}", val, rval);
        }
        */
    }
  
    // let mut flash = SpiNand::new(spi);

    println!("Oreboot read flash ID = {:?}", flash.read_id()).ok();

    // let mut page = [0u8; 256];
    // flash.copy_into(0, &mut page);

    let spi = flash.free();
    let (_spi, _pins) = spi.free();

    unsafe {
        for _ in 0..10_000_000 {
            //println!("💓");
            core::arch::asm!("nop");
            //println!("☢️💓☢️");
        }
    }
    let addr = RAM_BASE;
    println!("Run payload at {:#?}", addr);
    unsafe {
        let f: unsafe extern "C" fn() = transmute(addr);
        println!("💓");
        //f();
        // let f = transmute::<usize, EntryPoint>(addr);
        // f(0, 0);
        println!("☢️💓☢️");
    }
  
    println!("OREBOOT").ok();
    println!("Test succeeded! 🦀").ok();
    // light up led
    //let mut pb5 = gpio.portb.pb5.into_output();
    
    unsafe {
        for _ in 0..1_000_000 {
            pb0.set_high().unwrap();
            core::arch::asm!("nop");
            pb1.set_high().unwrap();
            core::arch::asm!("nop");
            pb1.set_low().unwrap();

            pb0.set_low().unwrap();
            //core::arch::asm!("nop");
            pb1.set_high().unwrap();
            pb1.set_low().unwrap();
            
        }
    }   
    //pb5.set_low().unwrap();
    //let mut line1 = String::new();
    println!("Enter the desired frequency :");
    //let b1 = std::io::stdin().read_line(&mut line1).unwrap();
    //let b2 = read_line(&mut line1).unwrap();
    //println!("Hello , you requested for : {} Hz", line1);
    //println!("no of bytes read , {}", b1);

}

// should jump to dram but not reach there
extern "C" fn cleanup() -> ! {
    loop {
        unsafe { asm!("wfi") };
    }
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    if let Some(location) = info.location() {
        println!("panic occurred in file '{}' at line {}",
            location.file(),
            location.line(),
        ).ok();
    } else {
        println!("panic occurred but can't get location information...").ok();
    };
    loop {
        core::hint::spin_loop();
    }
}
