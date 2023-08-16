//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
#![deny(unsafe_code)]
#![deny(warnings)]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::InputPin;
use hal::{
    entry,
    gpio::{bank0::Gpio13, FunctionI2c, FunctionSio, Pin, PullUp, SioInput},
};
use panic_probe as _;
use rp2040_hal as hal;

mod spi_pio;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use sparkfun_pro_micro_rp2040 as bsp;

use fugit::RateExtU32;

use hal::{clocks::Clock, pac, watchdog::Watchdog};

use crate::spi_pio::{spi_pio_init, PioCfg};

#[allow(unsafe_code)]
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
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

    let mut _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin = pins.gpio16.into_function::<FunctionI2c>();
    let scl_pin = pins.gpio17.into_function::<FunctionI2c>();

    let mut _i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let cs_pin: Pin<Gpio13, FunctionSio<SioInput>, PullUp> = pins.gpio13.into_pull_up_input();

    let cfg = PioCfg {
        pio: pac.PIO0,
        cs_pin: &cs_pin,
        cipo_pin: pins.gpio10,
        copi_pin: pins.gpio11,
        sck_pin: pins.gpio12,
    };

    let mut pios = spi_pio_init(cfg, &mut pac.RESETS);

    // spi_pio_init(pac.PIO0, &mut pac.RESETS);

    let mut read_data = [0u8; 256];
    let mut write_data = [0u8; 256];

    _delay.delay_ms(50);

    loop {
        // Wait for CS to go high
        while cs_pin.is_low().unwrap() {}
        pios.bits_sm.restart();
        pios.bits_sm.clear_fifos();
        // TODO Clear RX buffer
        // Wait for CS to go low
        while cs_pin.is_high().unwrap() {}

        defmt::info!("Went High");

        let mut idx: u8 = 0;
        while cs_pin.is_low().unwrap() {
            match pios.bits_rx.read() {
                Some(d) => {
                    read_data[idx as usize] = d as u8;
                    idx = idx.wrapping_add(1);
                }
                None => {}
            }
        }
        defmt::info!("Read {} {}", idx, read_data[..idx as usize]);

        // Do I2C Stuff
        write_data[0] = 8;
        write_data[1] = 9;
        write_data[2] = 10;
        write_data[3] = 11;
        idx = 0;

        // Set data ready

        // Wait for high
        while cs_pin.is_low().unwrap() {}
        pios.bits_sm.restart();
        pios.bits_sm.clear_fifos();

        // TODO Clear TX Buffer

        // Load buffer
        pios.bits_tx.write_u8_replicated(write_data[idx as usize]);
        idx += 1;

        // Wait for CS to go low
        while cs_pin.is_high().unwrap() {}

        while cs_pin.is_low().unwrap() {
            // Add more bytes, increment if successfull
            if pios.bits_tx.write_u8_replicated(write_data[idx as usize]) {
                idx = idx.wrapping_add(1);
            }
        }

        defmt::info!("Write {} {}", idx, write_data[..idx as usize]);
    }

    // // These are implicitly used by the spi driver if they are in the correct mode
    // let _spi_mosi = pins.gpio12.into_mode::<bsp::hal::gpio::FunctionSpi>();
    // let _spi_miso = pins.gpio11.into_mode::<bsp::hal::gpio::FunctionSpi>();
    // let _spi_sclk = pins.gpio10.into_mode::<bsp::hal::gpio::FunctionSpi>();
    // let spi = spi::Spi::<_, _, 8>::new(pac.SPI1);

    // let mut cs = pins
    //     .gpio13
    //     .into_push_pull_output_in_state(hal::gpio::PinState::High);

    // let mut spi = spi.init(
    //     &mut pac.RESETS,
    //     clocks.peripheral_clock.freq(),
    //     1.MHz(),
    //     &MODE_3,
    // );

    // loop {
    //     cs.set_low();
    //     spi.write(&[1, 2, 3, 4]).unwrap();
    //     cs.set_high();
    //     _delay.delay_ms(100);
    // }

    // //let pio = pac.PIO0.split(&mut pac.RESETS);
    // //let ms: hal::pio::UninitStateMachine<(pac::PIO0, hal::pio::SM0)> = pio.1;

    // //spi_pio_init();

    // let write_bytes: [u8; 1] = [0x07];
    // let mut read_bytes: [u8; 1] = [0];

    // match i2c.write_read(0x52, &write_bytes, &mut read_bytes) {
    //     Ok(()) => {
    //         info!("Success!");
    //     }
    //     Err(e) => {
    //         defmt::info!("Error {}", e);
    //     }
    // }

    // // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // // on-board LED, it might need to be changed.
    // // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // // LED to one of the GPIO pins, and reference that pin here.
    // let mut led_pin = pins.gpio13.into_push_pull_output();

    // loop {
    //     info!("on!");
    //     led_pin.set_high().unwrap();
    //     delay.delay_ms(500);
    //     info!("off!");
    //     led_pin.set_low().unwrap();
    //     delay.delay_ms(500);
    // }
}

// End of file
