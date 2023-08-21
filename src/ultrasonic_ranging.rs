#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;

use embedded_hal::PwmPin;
use embedded_hal::digital::v2::{InputPin, OutputPin};

use rp2040_hal::clocks::Clock;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use serial_write::Writer;

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

    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();

    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut pwm_silces = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let pwm = &mut pwm_silces.pwm1;
    pwm.enable();
    pwm.set_top(24999);
    pwm.set_div_int(100);
    pwm.set_div_frac(0);

    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio3);

    // Set an input from a switch to gpio5
    let switch = pins.gpio5.into_pull_up_input();
    // Set an input from a ultrasonic ranging sensor (echo) to gpio16
    let echo = pins.gpio16.into_pull_down_input();
    // Set an output to a ultrasonic ranging sensor (trigger) from gpio17
    let mut trigger = pins.gpio17.into_push_pull_output();

    let mut writer = Writer::new();

    // Switch on/off
    let mut switch_flg = false;
    channel.set_duty(0);
    loop {
        delay.delay_ms(5);
        let _ = usb_dev.poll(&mut [&mut serial]);

        // Switch on
        if switch.is_low().ok().unwrap() {
            if switch_flg {
                continue;
            } else {
                // Trigger ultrasonic pulse
                trigger.set_low().ok().unwrap();
                delay.delay_us(2);
                trigger.set_high().ok().unwrap();
                delay.delay_us(10);
                trigger.set_low().ok().unwrap();

                // Measure the time it took for the pulse to come back
                let mut time_low = 0;
                let mut time_high = 0;
                while echo.is_low().ok().unwrap() {
                    time_low = timer.get_counter().ticks();
                }
                while echo.is_high().ok().unwrap() {
                    time_high = timer.get_counter().ticks();
                }
                let time = time_high - time_low;

                // Convert the time to the distance (cm)
                let distance = time as f64 * 0.0343 / 2.0;

                // Display the distance
                let _ = writer.write_f64(distance, 2, &mut serial);
                let _ = writer.writeln_str("cm", &mut serial);

                switch_flg = true;

                // Adjust the brightness of the LED according to the distance
                if distance > 100.0 {
                    channel.set_duty(1000);
                } else {
                    channel.set_duty((64535 as f64 * ((100.0 - distance) / 100.0)) as u16 + 1000);
                }
            }
        } else {
            switch_flg = false;
        }
    }
}

// End of file