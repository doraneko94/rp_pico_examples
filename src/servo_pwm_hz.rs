#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;

use embedded_hal::PwmPin;
use rp2040_hal::clocks::Clock;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM1
    let pwm = &mut pwm_slices.pwm1;
    pwm.enable();
    // Set the PWM frequency to 50Hz
    pwm.set_top(24999);
    pwm.set_div_int(100);
    pwm.set_div_frac(0);

    // Output channel B on PWM1 to the servo pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio3);

    // Infinite loop, rotating the servo left or right
    // SG90-HV datasheet
    // Right Max: 25000 *  5.0% = 1250
    // Stop     : 25000 *  7.5% = 1875
    // Left  Max: 25000 * 10.0% = 2500
    loop {
        channel.set_duty(1250);
        delay.delay_ms(5000);
        channel.set_duty(1875);
        delay.delay_ms(5000);
        channel.set_duty(2500);
        delay.delay_ms(5000);
    }
}

// End of file