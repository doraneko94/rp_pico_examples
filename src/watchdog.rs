// This code was inspired by https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/watchdog.rs

#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;

use embedded_hal::digital::v2::OutputPin;

use rp2040_hal::Timer;
use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
use fugit::ExtU32;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Start Watchdog tick generation at 120 MHz
    watchdog.enable_tick_generation((XTAL_FREQ_HZ / 1_000_000) as u8);

    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);

    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_high().ok().unwrap();
    delay_ms(2000, &timer);

    // Activate Watchdog with time limit of 1.05 seconds
    watchdog.start(1_050.millis());

    // Blink once a second for 5 seconds, refreshing the watchdog timer once a second to avoid a reset
    for _ in 1..=5 {
        led_pin.set_low().unwrap();
        delay_ms(500, &timer);
        led_pin.set_high().unwrap();
        delay_ms(500, &timer);
        watchdog.feed();
    }

    // Blink 10 times per second, not feeding the watchdog.
    // The processor should reset in 1.05 seconds, or 5 blinks time
    loop {
        led_pin.set_low().unwrap();
        delay_ms(100, &timer);
        led_pin.set_high().unwrap();
        delay_ms(100, &timer);
    }
}

// Since the `delay` methods were implemented for `Timer` in Issue #138 of A, 
// a similar function should be used.
fn delay_ms(ms: u32, timer: &Timer) {
    let us = (ms * 1_000) as u64;
    let start = timer.get_counter().ticks();
    while timer.get_counter().ticks() - start < us {}
}

// End of file