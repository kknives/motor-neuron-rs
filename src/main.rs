//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::pio::{StateMachine, UninitStateMachine},
};
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use pio_proc::pio_file;
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
    loop {
        tx0.write(0x31);
        tx1.write(0x31);
        tx2.write(0x31);
        tx3.write(0x31);
        tx4.write(0x31);
        tx5.write(0x31);
        delay.delay_ms(100);
        // cortex_m::asm::wfi();
    }
}

// End of file
