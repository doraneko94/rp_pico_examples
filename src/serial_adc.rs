#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;

// Use ADC.
use embedded_hal::adc::OneShot;

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

    // ADC settings.
    let mut adc = hal::adc::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin = pins.gpio28.into_floating_input();

    // Set a serial writer.
    let mut writer = Writer::new();

    // Infinite loop, measuring voltage.
    loop {
        for i in 0..200 {
            delay.delay_ms(5);
            let _ = usb_dev.poll(&mut [&mut serial]);
            if i == 199 {
                let r: u16 = adc.read(&mut adc_pin).unwrap();
                let v = r as f64 * 3.3 / 4095.0;
                let _ = writer.write_f64(v, 2, &mut serial);
                let _ = writer.writeln_str(" V", &mut serial);
            }
        }
    }
}

// End of file