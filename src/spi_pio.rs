use pio::Instruction;
use rp2040_hal::{
    gpio::{FunctionPio0, Pins},
    pac::{self, PIO0},
    pio::{PIOBuilder, PIOExt, PinDir},
};

const CS_PIN: u8 = 1;
const CIPO_PIN: u8 = 2;
const COPI_PIN: u8 = 3;

pub fn spi_pio_init(pio: PIO0, pins: &mut Pins, resets: &mut pac::RESETS) {
    let (mut pio, sm0, sm1, _, _) = pio.split(resets);

    let cs_program = pio_proc::pio_asm!(
        ".side_set 1 pindirs",
        "wait 0 pin 0 side 0", // wait for falling edge of cs pin
        "irq set 7 side 1",
        "irq set 1 side 1",    // trigger falling edge irq",
        "wait 1 pin 0 side 1", // wait for rising edge of cs pin",
        "irq set 2 side 0",    //trigger rising edge irq",
    );

    let cs_installed = pio.install(&cs_program.program).unwrap();
    let (mut cs_sm, cs_rx, cs_tx) = PIOBuilder::from_program(cs_installed)
        .in_pin_base(CS_PIN)
        .side_set_pin_base(CIPO_PIN)
        .build(sm0);

    cs_sm.set_pindirs([(CIPO_PIN, PinDir::Input)]);

    let program = pio_proc::pio_asm!(
        "wait_falling:",
        "wait 0 pin 1", // Wait for falling clock edge
        "initial_loop:",
        "pull ifempty noblock", // Pull if there is data
        "out pins 1",           // Write CIPO
        "wait 1 pin 1",         // Wait for rising clock edge
        "in pins 1",            // Read COPI
        "push iffull noblock",  // Push if we've read 8 bits, but don't block if fifo is full
        ".wrap",
        ".wrap_target",
        "jmp wait_falling",
        "public initial_check:",
        "jmp y-- wait_falling",
        "irq set 0", // Trigger data set irq
        "jmp wait_falling",
    );

    let installed = pio.install(&program.program).unwrap();
    let (mut bits_sm, bits_rx, bits_tx) = PIOBuilder::from_program(installed)
        .in_pin_base(COPI_PIN)
        .out_pins(CIPO_PIN, 1)
        .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
        .in_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
        .autopull(false)
        .autopush(false)
        .push_threshold(8)
        .pull_threshold(8)
        .build(sm1);
    let cipo = pins.gpio2.into_mode::<FunctionPio0>();
    assert_eq!(cipo.id().num, CIPO_PIN);

    bits_tx.write(0xFF << 24);
    bits_sm.exec_instruction(Instruction {
        operands: pio::InstructionOperands::PULL {
            if_empty: false,
            block: true,
        },
        delay: 0,
        side_set: None,
    });
    bits_sm.exec_instruction(Instruction {
        operands: pio::InstructionOperands::MOV {
            destination: pio::MovDestination::X,
            op: pio::MovOperation::None,
            source: pio::MovSource::OSR,
        },
        delay: 0,
        side_set: None,
    });
}

pub fn wait_for_data() {}
