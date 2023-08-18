use pio::Instruction;
use rp2040_hal::gpio::{FunctionSio, Pin, PinId, PullNone, PullUp, SioInput, ValidFunction};
use rp2040_hal::pio::{Running, Rx, StateMachine, Tx, PIO, SM0, SM1};
use rp2040_hal::{
    gpio::FunctionPio0,
    pac::{self},
    pio::{PIOBuilder, PIOExt, PinDir},
};

pub struct PioCfg<'a, P: PIOExt, CS: PinId, CIPO: PinId, COPI: PinId, SCK: PinId> {
    pub pio: P,
    pub cs_pin: &'a Pin<CS, FunctionSio<SioInput>, PullUp>,
    pub cipo_pin: Pin<CIPO, FunctionSio<SioInput>, PullNone>,
    pub copi_pin: Pin<COPI, FunctionSio<SioInput>, PullNone>,
    pub sck_pin: Pin<SCK, FunctionSio<SioInput>, PullNone>,
}

pub struct SPIPio<P: PIOExt> {
    pub cs_sm: StateMachine<(P, SM0), Running>,
    pub bits_sm: StateMachine<(P, SM1), Running>,
    pub bits_tx: Tx<(P, SM1)>,
    pub bits_rx: Rx<(P, SM1)>,
    pub pio: PIO<P>,
}

pub fn spi_pio_init<
    P: PIOExt,
    CS: PinId,
    CIPO: PinId + ValidFunction<FunctionPio0>,
    COPI: PinId,
    SCK: PinId,
>(
    cfg: PioCfg<P, CS, CIPO, COPI, SCK>,
    resets: &mut pac::RESETS,
) -> SPIPio<P> {
    let (mut pio, sm0, sm1, _, _) = cfg.pio.split(resets);

    let cs_program = pio_proc::pio_asm!(
        ".side_set 1 pindirs",
        "wait 0 pin 0 side 0", // wait for falling edge of cs pin
        "irq clear 7 side 1",  // set loop irq
        "wait 1 pin 0 side 1", // wait for rising edge of cs pin
    );

    let cs_installed = pio.install(&cs_program.program).unwrap();
    let (mut cs_sm, _, _) = PIOBuilder::from_program(cs_installed)
        .in_pin_base(cfg.cs_pin.id().num)
        .side_set_pin_base(cfg.cipo_pin.id().num)
        .build(sm0);

    cs_sm.set_pindirs([(cfg.cipo_pin.id().num, PinDir::Input)]);

    let cs_sm = cs_sm.start();

    let program = pio_proc::pio_asm!(
        "irq wait 7",
        "loop:",
        "wait 0 pin 1",         // Wait for falling clock edge
        "pull ifempty noblock", // Pull if there is data
        "out pins 1",           // Write CIPO
        "wait 1 pin 1",         // Wait for rising clock edge
        "in pins 1",            // Read COPI
        "push iffull noblock",  // Push if we've read 8 bits, but don't block if fifo is full
        "jmp loop"
    );

    let installed = pio.install(&program.program).unwrap();
    let (mut bits_sm, bits_rx, mut bits_tx) = PIOBuilder::from_program(installed)
        .in_pin_base(cfg.copi_pin.id().num)
        .out_pins(cfg.cipo_pin.id().num, 1)
        .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
        .in_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
        .autopull(false)
        .autopush(false)
        .push_threshold(8)
        .pull_threshold(8)
        .build(sm1);
    cfg.cipo_pin.into_function::<FunctionPio0>();

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

    let bits_sm = bits_sm.start();

    return SPIPio {
        cs_sm,
        bits_sm,
        bits_tx,
        bits_rx,
        pio,
    };
}
