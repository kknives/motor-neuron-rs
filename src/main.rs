//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_halt as _;

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use heapless::String;
use pio_proc::pio_file;
use postcard::{from_bytes, to_slice};
use fugit::{Duration, ExtU32};
use rotary_encoder_embedded::{standard::StandardMode, RotaryEncoder, Direction};
use serde::{Deserialize, Serialize};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio,
    gpio::pin::{bank0, Pin},
    pac,
    pac::interrupt,
    pio::PIOExt,
    pio::{StateMachineIndex, Tx, SM0, SM1, SM2, SM3},
    sio::Sio,
    timer::Alarm,
    watchdog::Watchdog,
};

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
enum Operation {
    KeepAlive,
    SabertoothWrite(u8, u8),
    SmartelexWrite(u8, [u8; 5]),
    EncoderRead,
    PwmWrite(u8, u16),
}

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
impl Operation {
    fn handle_operation<'a, P0: PIOExt, P1: PIOExt, B: usb_device::bus::UsbBus>(
        self,
        usb_serial: &mut SerialPort<'a, B>,
        sabertooth0: &mut Tx<(P0, SM0)>,
        sabertooth1: &mut Tx<(P0, SM1)>,
        sabertooth2: &mut Tx<(P0, SM2)>,
        sabertooth3: &mut Tx<(P0, SM3)>,
        smartelex: &mut Tx<(P1, SM0)>,
    ) {
        match self {
            Operation::SabertoothWrite(tx_id, value) => {
                match tx_id {
                    0 => {
                        let _ = sabertooth0.write(value.into());
                    }
                    1 => {
                        let _ = sabertooth1.write(value.into());
                    }
                    2 => {
                        let _ = sabertooth2.write(value.into());
                    }
                    3 => {
                        let _ = sabertooth3.write(value.into());
                    }
                    _ => {
                        // Do nothing
                    }
                }
            }
            Operation::SmartelexWrite(tx_id, value) => {
                if let tx_id = 4 {
                    value.iter().for_each(|v| {
                        let _ = smartelex.write(*v as u32);
                    });
                }
            }
            Operation::EncoderRead => {
                cortex_m::interrupt::free(|cs| {
                    let mut encoder_positions = ENCODER_POSITIONS.borrow(cs).borrow_mut();
                    let encoder_positions = encoder_positions.as_mut().unwrap();
                    encoder_positions.iter().for_each(|v| {
                        let _ = usb_serial.write(&v.to_le_bytes());
                    });
                });
            }
            _ => {
                // Do nothing
            }
        }
    }
}
type EncoderInputPin<P> = gpio::pin::Pin<P, gpio::pin::Input<gpio::pin::PullUp>>;
// gpio5, gpio8, gpio9, gpio10, gpio11, gpio12, gpio13, gpio14, gpio15, gpio16
type EncoderTuple = (
    RotaryEncoder<StandardMode, EncoderInputPin<bank0::Gpio5>, EncoderInputPin<bank0::Gpio8>>,
    RotaryEncoder<StandardMode, EncoderInputPin<bank0::Gpio9>, EncoderInputPin<bank0::Gpio10>>,
    RotaryEncoder<StandardMode, EncoderInputPin<bank0::Gpio11>, EncoderInputPin<bank0::Gpio12>>,
    RotaryEncoder<StandardMode, EncoderInputPin<bank0::Gpio13>, EncoderInputPin<bank0::Gpio14>>,
    RotaryEncoder<StandardMode, EncoderInputPin<bank0::Gpio15>, EncoderInputPin<bank0::Gpio16>>,
);
static ENCODERS: Mutex<RefCell<Option<EncoderTuple>>> = Mutex::new(RefCell::new(None));
static ENCODER_POSITIONS: Mutex<RefCell<Option<[i8; 5]>>> = Mutex::new(RefCell::new(None));
static ALARM: Mutex<RefCell<Option<bsp::hal::timer::Alarm0>>> = Mutex::new(RefCell::new(None));
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

    // Pins 7, 11, 12, 14, 15, 16, 17, 19, 20, 21 are encoder inputs
    let pin_d7 = pins.gpio5.into_pull_up_input();
    let pin_d11 = pins.gpio8.into_pull_up_input();
    let pin_d12 = pins.gpio9.into_pull_up_input();
    let pin_d14 = pins.gpio10.into_pull_up_input();
    let pin_d15 = pins.gpio11.into_pull_up_input();
    let pin_d16 = pins.gpio12.into_pull_up_input();
    let pin_d17 = pins.gpio13.into_pull_up_input();
    let pin_d19 = pins.gpio14.into_pull_up_input();
    let pin_d20 = pins.gpio15.into_pull_up_input();
    let pin_d21 = pins.gpio16.into_pull_up_input();

    let mut rotary_encoders = (
        RotaryEncoder::new(pin_d7, pin_d11).into_standard_mode(),
        RotaryEncoder::new(pin_d12, pin_d14).into_standard_mode(),
        RotaryEncoder::new(pin_d15, pin_d16).into_standard_mode(),
        RotaryEncoder::new(pin_d17, pin_d19).into_standard_mode(),
        RotaryEncoder::new(pin_d20, pin_d21).into_standard_mode(),
    );

    let uart_pins = [0, 1, 2, 3, 4];
    let _uart_gpios = (
        pins.gpio0.into_mode::<bsp::hal::gpio::FunctionPio0>(),
        pins.gpio1.into_mode::<bsp::hal::gpio::FunctionPio0>(),
        pins.gpio2.into_mode::<bsp::hal::gpio::FunctionPio0>(),
        pins.gpio3.into_mode::<bsp::hal::gpio::FunctionPio0>(),
        pins.gpio4.into_mode::<bsp::hal::gpio::FunctionPio1>(),
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

    let mut timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm = timer.alarm_0().unwrap();
    let mut encoder_positions = [0; 5];
    cortex_m::interrupt::free(|cs| {
        ENCODERS.borrow(cs).replace(Some(rotary_encoders));
        ENCODER_POSITIONS.borrow(cs).replace(Some(encoder_positions));
        alarm.schedule(1111.micros()).unwrap();
        alarm.enable_interrupt();
        ALARM.borrow(cs).replace(Some(alarm));
    });

    #[allow(unsafe_code)]
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    loop {
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
                    op.handle_operation(&mut serial, &mut tx0, &mut tx1, &mut tx2, &mut tx3, &mut tx4);
                }
            }
        }
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        let mut alarm = ALARM.borrow(cs).borrow_mut();
        let mut alarm = alarm.as_mut().unwrap();
        let mut encoders = ENCODERS.borrow(cs).borrow_mut();
        let mut encoders = encoders.as_mut().unwrap();
        let mut encoder_positions = ENCODER_POSITIONS.borrow(cs).borrow_mut();
        let mut encoder_positions = encoder_positions.as_mut().unwrap();

        encoders.0.update();
        match encoders.0.direction() {
            Direction::Clockwise => {
                encoder_positions[0] += 1;
            }
            Direction::Anticlockwise => {
                encoder_positions[0] -= 1;
            }
            Direction::None => {}
        }
        encoders.1.update();
        match encoders.1.direction() {
            Direction::Clockwise => {
                encoder_positions[1] += 1;
            }
            Direction::Anticlockwise => {
                encoder_positions[1] -= 1;
            }
            Direction::None => {}
        }
        encoders.2.update();
        match encoders.2.direction() {
            Direction::Clockwise => {
                encoder_positions[2] += 1;
            }
            Direction::Anticlockwise => {
                encoder_positions[2] -= 1;
            }
            Direction::None => {}
        }
        encoders.3.update();
        match encoders.3.direction() {
            Direction::Clockwise => {
                encoder_positions[3] += 1;
            }
            Direction::Anticlockwise => {
                encoder_positions[3] -= 1;
            }
            Direction::None => {}
        }
        encoders.4.update();
        match encoders.4.direction() {
            Direction::Clockwise => {
                encoder_positions[4] += 1;
            }
            Direction::Anticlockwise => {
                encoder_positions[4] -= 1;
            }
            Direction::None => {}
        }

        alarm.clear_interrupt();
        alarm.schedule(1111.micros()).unwrap();
    });
}
// End of file
