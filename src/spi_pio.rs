use core::cell::RefCell;
use core::mem;

use critical_section::Mutex;
use pio::Instruction;
use rp2040_hal::gpio::{FunctionSio, Pin, PinId, PullNone, PullUp, SioInput, ValidFunction};
use rp2040_hal::pac::{PIO0, PIO1};
use rp2040_hal::pio::{Running, Rx, Stopped, Tx, PIO, SM0, SM1, StateMachineGroup2};
use rp2040_hal::{
    gpio::FunctionPio0,
    pac::interrupt,
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

pub enum PioSm<P: PIOExt> {
    None,
    Running(StateMachineGroup2<P, SM0, SM1, Running>),
    Stopped(StateMachineGroup2<P, SM0, SM1, Stopped>),
}

impl<P:PIOExt> PioSm<P> {
    pub fn to_running(&mut self) {
        let exist = mem::replace(self, Self::None);
        let new = PioSm::Running(match exist {
            PioSm::None => panic!(),
            PioSm::Running(s) => s,
            PioSm::Stopped(s) => s.start(),
        });
        _ = mem::replace(self, new);
    }
}

pub struct SPIPio<P: PIOExt> {
    pub sm: PioSm<P>,
    pub bits_tx: Tx<(P, SM1)>,
    pub bits_rx: Rx<(P, SM1)>,
    pub pio: PIO<P>,
}

type SPIPio0 = SPIPio<PIO0>;
type SPIPio1 = SPIPio<PIO1>;

static SPI_PIO_0: Mutex<RefCell<Option<SPIPio0>>> = Mutex::new(RefCell::new(None));
static SPI_PIO_1: Mutex<RefCell<Option<SPIPio1>>> = Mutex::new(RefCell::new(None));

pub struct SPIPioHelper<P: PIOExt + 'static> {
    pio: &'static Mutex<RefCell<Option<SPIPio<P>>>>
}

impl<P: PIOExt + 'static> SPIPioHelper<P> {
    pub fn start_state_machines(&mut self) {
        critical_section::with(|cs| {
            if let Some(s) = self.pio.borrow_ref_mut(cs).as_mut() {
                s.sm.to_running();
            }
        });
    }

    // Add way to stop safely
}

pub fn spi_pio_init_0<
    CS: PinId,
    CIPO: PinId + ValidFunction<FunctionPio0>,
    COPI: PinId,
    SCK: PinId,
>(
    cfg: PioCfg<PIO0, CS, CIPO, COPI, SCK>,
    resets: &mut pac::RESETS,
) -> SPIPioHelper<PIO0> {
    let pio = spi_pio_init(cfg, resets);

    critical_section::with(|cs| {
        SPI_PIO_0.borrow(cs).replace(Some(pio));
    });

    #[allow(unsafe_code)]
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
    }

    return SPIPioHelper {pio: &SPI_PIO_0};
}

pub fn _spi_pio_init_1<
    CS: PinId,
    CIPO: PinId + ValidFunction<FunctionPio0>,
    COPI: PinId,
    SCK: PinId,
>(
    cfg: PioCfg<PIO1, CS, CIPO, COPI, SCK>,
    resets: &mut pac::RESETS,
) {
    let pio = spi_pio_init(cfg, resets);

    critical_section::with(|cs| {
        SPI_PIO_1.borrow(cs).replace(Some(pio));
    });

    #[allow(unsafe_code)]
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::PIO1_IRQ_0);
    }
}

fn spi_pio_init<
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
        "irq set 1 side 1",    // trigger falling edge IRQ
        "wait 1 pin 0 side 1", // wait for rising edge of cs pin
        "irq set 2 side 0",    // trigger rising edge irq
    );

    let cs_installed = pio.install(&cs_program.program).unwrap();
    let (mut cs_sm, _, _) = PIOBuilder::from_program(cs_installed)
        .in_pin_base(cfg.cs_pin.id().num)
        .side_set_pin_base(cfg.cipo_pin.id().num)
        .build(sm0);

    cs_sm.set_pindirs([(cfg.cipo_pin.id().num, PinDir::Input)]);

    // Setup IRQs
    pio.irq0().enable_sm_interrupt(1);
    pio.irq0().enable_sm_interrupt(2);

    let program = pio_proc::pio_asm!(
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

    let sm: rp2040_hal::pio::StateMachineGroup2<P, SM0, SM1, Stopped> = cs_sm.with(bits_sm);

    SPIPio {
        sm: PioSm::Stopped(sm),
        bits_tx,
        bits_rx,
        pio,
    }
}

fn handle_irq<P: PIOExt>(pio: &mut SPIPio<P>) {
    let irqs = pio.pio.get_irq_raw();
    if (irqs & (1 << 1)) != 0 {
        // Falling IRQ
        defmt::println!("Falling IRQ");
        pio.pio.clear_irq(1 << 1);
    }
    if (irqs & (1 << 2)) != 0 {
        // Rising IRQ
        defmt::println!("Raising IRQ");
        pio.pio.clear_irq(1 << 2);
    }
}

#[interrupt]
fn PIO0_IRQ_0() {
    critical_section::with(|cs| {
        if let Some(s) = SPI_PIO_0.borrow_ref_mut(cs).as_mut() {
            handle_irq(s);
        }
    });
}

#[interrupt]
fn PIO1_IRQ_0() {
    critical_section::with(|cs| {
        if let Some(s) = SPI_PIO_1.borrow_ref_mut(cs).as_mut() {
            handle_irq(s);
        }
    });
}
