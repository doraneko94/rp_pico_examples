#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;

use embedded_hal::digital::v2::OutputPin;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let _clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio3.into_push_pull_output();

    // Create a timer
    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);

    loop {
        led_pin.set_high().unwrap();
        let start = timer.get_counter().ticks();
        while timer.get_counter().ticks() - start < 500_000 {}
        led_pin.set_low().unwrap();
        let start = timer.get_counter().ticks();
        while timer.get_counter().ticks() - start < 500_000 {}
    }
}

// End of file