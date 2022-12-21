//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_halt as _;

use serde::{Serialize, Deserialize};
use postcard::{to_slice, from_bytes};
use core::fmt::Write;
use heapless::String;
use pio_proc::pio_file;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    pio::PIOExt,
    sio::Sio,
    watchdog::Watchdog,
};

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
enum Operation {
    KeepAlive,
    SabertoothWrite(u8, u8),
    SmartelexWrite(u8, [u8; 5]),
    EncoderRead(u8, u8),
    PwmWrite(u8, u16),
}

// Send help please
struct UARTPIOBuilder<P: PIOExt>(bsp::hal::pio::PIOBuilder<P>);
impl<P: PIOExt> UARTPIOBuilder<P> {
    fn setup_pio_uart(
        clock_freq: u32,
        installed: bsp::hal::pio::InstalledProgram<P>,
        pin: u8,
    ) -> bsp::hal::pio::PIOBuilder<P> {
        bsp::hal::pio::PIOBuilder::from_program(installed)
            .set_pins(pin, 1)
            .out_pins(pin, 1)
            .autopull(false)
            .out_shift_direction(bsp::hal::pio::ShiftDirection::Right)
            .side_set_pin_base(pin)
            .clock_divisor(clock_freq as f32 / (8f32 * 9600f32))
            .buffers(bsp::hal::pio::Buffers::OnlyTx)
    }
}
#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = [0, 1, 2, 3, 4, 5];
    let _uart_gpios = (
        pins.gpio0.into_mode::<bsp::hal::gpio::FunctionPio0>(),
        pins.gpio1.into_mode::<bsp::hal::gpio::FunctionPio0>(),
        pins.gpio2.into_mode::<bsp::hal::gpio::FunctionPio0>(),
        pins.gpio3.into_mode::<bsp::hal::gpio::FunctionPio0>(),
        pins.gpio4.into_mode::<bsp::hal::gpio::FunctionPio1>(),
        pins.gpio5.into_mode::<bsp::hal::gpio::FunctionPio1>(),
    );
    // Setup UARTs on first PIO segment
    let pio_program = pio_proc::pio_file!("./src/uart_tx.pio", select_program("uart_tx"));
    let (mut pio0, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);
    let (mut working_sm, _, mut tx0) = UARTPIOBuilder::setup_pio_uart(
        clocks.system_clock.freq().to_Hz(),
        pio0.install(&pio_program.program).unwrap(),
        uart_pins[0],
    )
    .build(sm0);
    working_sm.set_pindirs([(uart_pins[0], bsp::hal::pio::PinDir::Output)]);
    working_sm.start();

    let (mut working_sm, _, mut tx1) = UARTPIOBuilder::setup_pio_uart(
        clocks.system_clock.freq().to_Hz(),
        pio0.install(&pio_program.program).unwrap(),
        uart_pins[1],
    )
    .build(sm1);
    working_sm.set_pindirs([(uart_pins[1], bsp::hal::pio::PinDir::Output)]);
    working_sm.start();

    let (mut working_sm, _, mut tx2) = UARTPIOBuilder::setup_pio_uart(
        clocks.system_clock.freq().to_Hz(),
        pio0.install(&pio_program.program).unwrap(),
        uart_pins[2],
    )
    .build(sm2);
    working_sm.set_pindirs([(uart_pins[2], bsp::hal::pio::PinDir::Output)]);
    working_sm.start();

    let (mut working_sm, _, mut tx3) = UARTPIOBuilder::setup_pio_uart(
        clocks.system_clock.freq().to_Hz(),
        pio0.install(&pio_program.program).unwrap(),
        uart_pins[3],
    )
    .build(sm3);
    working_sm.set_pindirs([(uart_pins[3], bsp::hal::pio::PinDir::Output)]);
    working_sm.start();

    // Setup UART on the other PIO block, only using 2 state machines this time
    let (mut pio1, sm0, sm1, _, _) = pac.PIO1.split(&mut pac.RESETS);
    let (mut working_sm, _, mut tx4) = UARTPIOBuilder::setup_pio_uart(
        clocks.system_clock.freq().to_Hz(),
        pio1.install(&pio_program.program).unwrap(),
        uart_pins[4],
    )
    .build(sm0);
    working_sm.set_pindirs([(uart_pins[4], bsp::hal::pio::PinDir::Output)]);
    working_sm.start();

    let (mut working_sm, _, mut tx5) = UARTPIOBuilder::setup_pio_uart(
        clocks.system_clock.freq().to_Hz(),
        pio1.install(&pio_program.program).unwrap(),
        uart_pins[5],
    )
    .build(sm1);
    working_sm.set_pindirs([(uart_pins[5], bsp::hal::pio::PinDir::Output)]);
    working_sm.start();

    // Setup USB serial
    let usb_bus = UsbBusAllocator::new(bsp::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with Raspberry Pi Vendor ID and CDC UART Pid
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2E8A, 0x000A))
        .manufacturer("Team Rudra")
        .product("R23 Motor Neuron")
        .serial_number("001")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    let timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;
    loop {
        // if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
        //     said_hello = true;
        //     let _ = serial.write(b"Hello, World!\r\n");

        //     let time = timer.get_counter().ticks();
        //     let mut text: String<64> = String::new();
        //     writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

        // }
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 256];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    let op: Operation = from_bytes(&buf[0..count]).unwrap();
                    match op {
                        Operation::SabertoothWrite ( tx_id, value ) => {
                            match tx_id {
                                0 => {
                                    let _ = tx0.write(value.into());
                                }
                                1 => {
                                    let _ = tx1.write(value.into());
                                }
                                2 => {
                                    let _ = tx2.write(value.into());
                                }
                                3 => {
                                    let _ = tx3.write(value.into());
                                }
                                4 => {
                                    let _ = tx4.write(value.into());
                                }
                                5 => {
                                    let _ = tx5.write(value.into());
                                }
                                _ => {
                                    // Do nothing
                                }
                            }
                        }
                        Operation::SmartelexWrite ( tx_id, value ) => {
                            match tx_id {
                                5 => {
                                    for i in value {
                                        let _ = tx5.write(i.into());
                                    }
                                }
                                _ => {
                                    // Do nothing
                                }
                            }
                        }
                        _ => {
                            // Do nothing
                        }
                    }
                }
            }
        }
        // tx0.write(0x31);
        // tx1.write(0x31);
        // tx2.write(0x31);
        // tx3.write(0x31);
        // tx4.write(0x31);
        // tx5.write(0x31);
        // cortex_m::asm::wfi();
    }
}

// End of file
