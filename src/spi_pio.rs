use core::cell::RefCell;

use critical_section::Mutex;
use pio::Instruction;
use rp2040_hal::gpio::{AnyPin, Disabled, Pin, PinId, PullDown};
use rp2040_hal::pac::interrupt;
use rp2040_hal::pac::interrupt::PIO0_IRQ_0;
use rp2040_hal::pio::StateMachine;
use rp2040_hal::{
    gpio::{FunctionPio0, Pins},
    pac::{self, PIO0},
    pio::{PIOBuilder, PIOExt, PinDir},
};

pub struct Pio0Cfg<CS: PinId, CIPO: PinId, COPI: PinId, SCK: PinId> {
    pub pio: PIO0,
    pub cs_pin: Pin<CS, Disabled<PullDown>>,
    pub cipo_pin: Pin<CIPO, Disabled<PullDown>>,
    pub copi_pin: Pin<COPI, Disabled<PullDown>>,
    pub sck_pin: Pin<SCK, Disabled<PullDown>>,
}

struct GlobalPio0 {
    pio: rp2040_hal::pio::PIO<PIO0>,
    cs_sm: StateMachine<(PIO0, rp2040_hal::pio::SM0), rp2040_hal::pio::Running>,
}

static GLOBAL_PIO_0: Mutex<RefCell<Option<GlobalPio0>>> = Mutex::new(RefCell::new(None));

pub fn spi_pio_init_0<CS: PinId, CIPO: PinId, COPI: PinId, SCK: PinId>(
    cfg: Pio0Cfg<CS, CIPO, COPI, SCK>,
    resets: &mut pac::RESETS,
) {
    let (mut pio, sm0, sm1, _, _) = cfg.pio.split(resets);

    let cs_program = pio_proc::pio_asm!(
        ".side_set 1 pindirs",
        "wait 0 pin 0 side 0", // wait for falling edge of cs pin
        "irq set 7 side 1",
        "irq set 1 side 1",    // trigger falling edge irq",
        "wait 1 pin 0 side 1", // wait for rising edge of cs pin",
        "irq set 2 side 0",    //trigger rising edge irq",
    );

    cfg.cs_pin.id();

    let cs_installed = pio.install(&cs_program.program).unwrap();
    let (mut cs_sm, _, _) = PIOBuilder::from_program(cs_installed)
        .in_pin_base(cfg.cs_pin.id().num)
        .side_set_pin_base(cfg.cipo_pin.id().num)
        .build(sm0);

    cs_sm.set_pindirs([(cfg.cipo_pin.id().num, PinDir::Input)]);

    let irq0 = pio.irq0();
    irq0.enable_sm_interrupt(1);
    irq0.enable_sm_interrupt(2);

    let cs_sm = cs_sm.start();

    critical_section::with(|crit| {
        GLOBAL_PIO_0
            .borrow(crit)
            .replace(Some(GlobalPio0 { pio, cs_sm }))
    });

    unsafe {
        pac::NVIC::unmask(PIO0_IRQ_0);
    }

    // let program = pio_proc::pio_asm!(
    //     "wait_falling:",
    //     "wait 0 pin 1", // Wait for falling clock edge
    //     "initial_loop:",
    //     "pull ifempty noblock", // Pull if there is data
    //     "out pins 1",           // Write CIPO
    //     "wait 1 pin 1",         // Wait for rising clock edge
    //     "in pins 1",            // Read COPI
    //     "push iffull noblock",  // Push if we've read 8 bits, but don't block if fifo is full
    //     ".wrap",
    //     ".wrap_target",
    //     "jmp wait_falling",
    //     "public initial_check:",
    //     "jmp y-- wait_falling",
    //     "irq set 0", // Trigger data set irq
    //     "jmp wait_falling",
    // );

    // let installed = pio.install(&program.program).unwrap();
    // let (mut bits_sm, bits_rx, bits_tx) = PIOBuilder::from_program(installed)
    //     .in_pin_base(COPI_PIN)
    //     .out_pins(CIPO_PIN, 1)
    //     .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
    //     .in_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
    //     .autopull(false)
    //     .autopush(false)
    //     .push_threshold(8)
    //     .pull_threshold(8)
    //     .build(sm1);
    // let cipo = pins.gpio2.into_mode::<FunctionPio0>();
    // assert_eq!(cipo.id().num, CIPO_PIN);

    // bits_tx.write(0xFF << 24);
    // bits_sm.exec_instruction(Instruction {
    //     operands: pio::InstructionOperands::PULL {
    //         if_empty: false,
    //         block: true,
    //     },
    //     delay: 0,
    //     side_set: None,
    // });
    // bits_sm.exec_instruction(Instruction {
    //     operands: pio::InstructionOperands::MOV {
    //         destination: pio::MovDestination::X,
    //         op: pio::MovOperation::None,
    //         source: pio::MovSource::OSR,
    //     },
    //     delay: 0,
    //     side_set: None,
    // });
}

fn stop_loops(pio: &mut GlobalPio0) {
    pio.cs_sm.restart();
}

fn prepare_for_next(pio: &mut GlobalPio0) {}

#[interrupt]
fn PIO0_IRQ_0() {
    static mut PIO: Option<GlobalPio0> = None;

    if PIO.is_none() {
        critical_section::with(|crit| *PIO = GLOBAL_PIO_0.borrow(crit).take());
    }

    if let Some(pio) = PIO {
        let irqs = pio.pio.get_irq_raw();
        if (irqs & (1u8 << 1)) != 0 {
            defmt::info!("Falling!");
            pio.pio.clear_irq(1u8 << 1);
        }
        if (irqs & (1u8 << 2)) != 0 {
            defmt::info!("Rising!");
            pio.pio.clear_irq(1u8 << 2);
        }
    }
}
