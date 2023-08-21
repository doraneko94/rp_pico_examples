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

    // Configure PWM2
    let pwm = &mut pwm_slices.pwm2;
    // pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM2 to the LED pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio5);

    let duty_max = channel.get_max_duty();
    // Infinite loop, fading LED up and down
    loop {
        delay.delay_ms(1000);
        channel.set_duty(duty_max);
        delay.delay_ms(1000);
        channel.set_duty(duty_max / 2);
        delay.delay_ms(1000);
        channel.set_duty(0);
    }
}

// End of file