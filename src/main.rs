//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{entry, hal::pio::UninitStateMachine, pac::pio0::SM};
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use pio_proc::pio_file;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    pio::PIOExt,
    sio::Sio,
    watchdog::Watchdog,
};

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
    let pio_program = pio_proc::pio_file!("./src/uart_tx.pio", select_program("uart_tx"));

    let (mut pio, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);
    // let pio0sm: [UninitStateMachine<SM: ValidStateMachine>; 4] = [sm0, sm1, sm2, sm3];
    let installed = pio.install(&pio_program.program).unwrap();

    // for (i, smi) in uart_pins.into_iter().take(4).zip(pio0sm.into_iter()) {
    //     let (mut sm, _, tx) = bsp::hal::pio::PIOBuilder::from_program(installed)
    //         .set_pins(i, 1)
    //         .out_shift_direction(bsp::hal::pio::ShiftDirection::Right)
    //         .side_set_pin_base(i)
    //         .clock_divisor_fixed_point(int, frac)
    //         .build(smi);
    //     sm.set_pindirs([(i, bsp::hal::pio::PinDir::Output)]);
    //     sm.start();
    // }
    let (mut sm, _, mut tx) = bsp::hal::pio::PIOBuilder::from_program(installed)
        .set_pins(3, 1)
        .out_pins(3, 1)
        .autopull(false)
        .out_shift_direction(bsp::hal::pio::ShiftDirection::Right)
        .side_set_pin_base(3)
        .clock_divisor(clocks.system_clock.freq().to_Hz() as f32 / (8f32 * 9600f32))
        // .clock_divisor_fixed_point(1, 160) // 125 MHz / (8*9600) in fixed point
        .buffers(bsp::hal::pio::Buffers::OnlyTx)
        .build(sm0);

    sm.set_pindirs([(3, bsp::hal::pio::PinDir::Output)]);
    sm.start();

    loop {
        tx.write(0x31);
        delay.delay_ms(100);
        // cortex_m::asm::wfi();
    }
}

// End of file
