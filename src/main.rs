//! # mains-sniffer-rs (Work in Progress)

//! This project aims to measure and log the mains frequency with a good resolution.
//! It is currently unfinished

#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::fmt::Write;
use cortex_m::peripheral::syst::SystClkSource;
use hal::clocks::ClocksManager;
use hal::fugit::{HertzU32, RateExtU32};
use hal::gpio;
use hal::gpio::Interrupt::EdgeLow;
use hal::gpio::{bank0::Gpio26, Pin, PullUp};
use hal::multicore::{Multicore, Stack};
use hal::pac;
use hal::pac::{interrupt, CorePeripherals, Peripherals};
use hal::pll::common_configs::PLL_USB_48MHZ;
use hal::pll::{setup_pll_blocking, PLLConfig};
use hal::uart::{DataBits, StopBits, UartConfig};
use hal::xosc::setup_xosc_blocking;
use hal::Clock;
use hal::{Sio, Watchdog};
use panic_halt as _;
use rp2040_hal as hal;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const SYSTICK_RELOAD: u32 = 0x00FF_FFFF;

const PLL_SYS_136MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1100),
    refdiv: 1,
    post_div1: 4,
    post_div2: 2,
};

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(mut in_pin: Pin<Gpio26, gpio::FunctionSioInput, PullUp>) -> () {
    // Get the second core's copy of the `CorePeripherals`, which are per-core.
    // Unfortunately, `cortex-m` doesn't support this properly right now,
    // so we have to use `steal`.
    let mut core = unsafe { CorePeripherals::steal() };
    let mut pac = unsafe { Peripherals::steal() };

    let mut sio = Sio::new(pac.SIO);

    // Trigger on the 'falling edge' of the input pin.
    in_pin.set_interrupt_enabled(EdgeLow, true);
    in_pin.clear_interrupt(EdgeLow);

    // SYSTICK Timer config
    core.SYST.set_reload(SYSTICK_RELOAD);
    core.SYST.clear_current();
    core.SYST.set_clock_source(SystClkSource::Core); // Sysclk

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    let mut n_count: u8 = 0;
    let mut sum: u32 = 0;
    loop {
        if !core.SYST.is_counter_enabled() {
            // Timer has been stopped by interrupt
            let count = cortex_m::peripheral::SYST::get_current();
            if count != 0 && count < 0x00FFFF00 {
                core.SYST.clear_current();
                sum += count;
                n_count += 1;
                if n_count == 25 {
                    sio.fifo.write_blocking(sum);
                    n_count = 0;
                    sum = 0;
                }
            }
        }
    }
}

/// Entry point to our application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.

#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Manual clock configuration
    let xosc = setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz())
        .ok()
        .unwrap();

    // Configure watchdog tick generation to tick over every microsecond
    watchdog.enable_tick_generation((12) as u8);

    let mut clocks = ClocksManager::new(pac.CLOCKS);

    let pll_sys = setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency(),
        PLL_SYS_136MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();
    let pll_usb = setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    clocks.init_default(&xosc, &pll_sys, &pll_usb);

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the GPIO pin that will be our input
    let mut in_pin = pins.gpio26.reconfigure();

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
    );
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    writeln!(uart, "Hello World!\r").unwrap();
    writeln!(uart, "{}\r", clocks.system_clock.freq().to_Hz()).unwrap();

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // Spawn core1_task on core1
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _task = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(in_pin);
    });

    writeln!(uart, "Alive\r").unwrap();

    loop {
        let sum = sio.fifo.read_blocking();
        writeln!(uart, "{}\r", sum).unwrap();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        let mut int_src = 0u32;
        // We need PROC1 here as this will be running on core1
        let ptr = 0x4001415Cu32 as *mut u32; //  IO_BANK0: PROC1_INTS3 Register
        unsafe {
            int_src = ptr.read_volatile();
        }
        if int_src == 0x400u32 {
            // ensures that the interrupt comes from a low edge on GPIO26
            let ptr = 0xE000E010u32 as *mut u32; // M0PLUS: SYST_CSR Register
            unsafe {
                let syst_csr = ptr.read_volatile();
                ptr.write(syst_csr ^ 0x1); // Toggle enable bit
            }
        }
        let ptr = 0x400140FCu32 as *mut u32; // IO_BANK0: INTR3 Register
        unsafe {
            ptr.write(0xFFFFFFFFu32); // yolo all bits (Clear interrupt so we don't jump in this routine immidiatley again)
        }
    });
}

// End of file
