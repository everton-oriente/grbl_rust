
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, PIO1, PIO2};
use embassy_rp::pio::{InterruptHandler as PioIrq, Pio, Config as PioConfig, ShiftConfig, ShiftDirection};
//use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
//use embassy_sync::mutex::Mutex;
use heapless::spsc::Queue;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _}; // RTT logging and panic handler

pub const AXIS_QUEUE_DEPTH: usize = 256; // change to 512 if desired

mod hw;
mod motion;



bind_interrupts!(struct Irqs {

    PIO0_IRQ_0 => PioIrq<PIO0>;
    PIO1_IRQ_0 => PioIrq<PIO1>;
    PIO2_IRQ_0 => PioIrq<PIO2>;
});

// Per-axis queues (X/Y enabled; Z/A/B/C declared but not used yet)
static Q_X: StaticCell<Queue<hw::Segment, AXIS_QUEUE_DEPTH>> = StaticCell::new();
static Q_Y: StaticCell<Queue<hw::Segment, AXIS_QUEUE_DEPTH>> = StaticCell::new();
// static Q_Z: StaticCell<Queue<Segment, AXIS_QUEUE_DEPTH>> = StaticCell::new();
// static Q_A: StaticCell<Queue<Segment, AXIS_QUEUE_DEPTH>> = StaticCell::new();
// static Q_B: StaticCell<Queue<Segment, AXIS_QUEUE_DEPTH>> = StaticCell::new();
// static Q_C: StaticCell<Queue<Segment, AXIS_QUEUE_DEPTH>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("RP2350B DMA+PIO stepper (6-axis skeleton; X/Y enabled)");

    let p = embassy_rp::init(Default::default());
    let pio0 = p.PIO0;
    let Pio {mut common, sm0: mut sm, ..} = Pio::new(pio0, Irqs);
    let pio1 = p.PIO1;
    let Pio {mut common, sm0: mut sm, ..} = Pio::new(pio1, Irqs);
    
    let mut pio2 = Pio::new(p.PIO2, Irqs);
    let pio2_sm0 = pio2.sm0;
    let common2 = pio2.common;

    // Common handles
    let c0 = Pio0.common();
    let c1 = Pio1.common();
    //let _c2 = pio2.Common();
    //let _c2 = pio2.Common();
    //let _c2 = pio2.Common();
    //let _c2 = pio2.Common();

    // Example pins for X/Y (edit to match your board)
    let x_step_pin: u8 = 6;
    let mut x_dir = Output::new(p.PIN_7, Level::Low);
    let mut x_en  = Output::new(p.PIN_8, Level::Low); // always enabled

    let y_step_pin: u8 = 10;
    let mut y_dir = Output::new(p.PIN_11, Level::Low);
    let mut y_en  = Output::new(p.PIN_12, Level::Low); // always enabled

    // PIO clocks ~150MHz, divider=1.0
    let pio_clk_hz: u32 = 150_000_000;

    // DMA channels for X/Y
    let dma_x = p.DMA_CH0;
    let dma_y = p.DMA_CH1;

    // Create X on PIO0.SM0; Y on PIO1.SM0
    let mut x = hw::Stepper::<PIO0, 0>::new(pio0, c0, x_step_pin, dma_x, x_dir, x_en, pio_clk_hz);
    let mut y = hw::Stepper::<PIO1, 0>::new(pio1, c1, y_step_pin, dma_y, y_dir, y_en, pio_clk_hz);

    // ---- Commented axes you can enable later ----
    // let z_step_pin: u8 = 14;
    // let mut z_dir = Output::new(p.PIN_15, Level::Low);
    // let mut z_en  = Output::new(p.PIN_16, Level::Low);
    // let dma_z = embassy_rp::dma::Channel::new(p.DMA_CH2);
    // let mut z = Stepper::<PIO0, 1>::new(pio0, c0, z_step_pin, dma_z, z_dir, z_en, pio_clk_hz);

    // let a_step_pin: u8 = 18;
    // let mut a_dir = Output::new(p.PIN_19, Level::Low);
    // let mut a_en  = Output::new(p.PIN_20, Level::Low);
    // let dma_a = embassy_rp::dma::Channel::new(p.DMA_CH3);
    // let mut a = Stepper::<PIO1, 1>::new(pio1, c1, a_step_pin, dma_a, a_dir, a_en, pio_clk_hz);

    // let b_step_pin: u8 = 22;
    // let mut b_dir = Output::new(p.PIN_23, Level::Low);
    // let mut b_en  = Output::new(p.PIN_24, Level::Low);
    // let dma_b = embassy_rp::dma::Channel::new(p.DMA_CH4);
    // let mut b = Stepper::<PIO2, 0>::new(pio2, _c2, b_step_pin, dma_b, b_dir, b_en, pio_clk_hz);

    // let c_step_pin: u8 = 26;
    // let mut c_dir = Output::new(p.PIN_27, Level::Low);
    // let mut c_en  = Output::new(p.PIN_28, Level::Low);
    // let dma_c = embassy_rp::dma::Channel::new(p.DMA_CH5);
    // let mut c = Stepper::<PIO2, 1>::new(pio2, _c2, c_step_pin, dma_c, c_dir, c_en, pio_clk_hz);

    // Queues
    let qx = Q_X.init(Queue::new());
    let qy = Q_Y.init(Queue::new());
    // let qz = Q_Z.init(Queue::new());
    // let qa = Q_A.init(Queue::new());
    // let qb = Q_B.init(Queue::new());
    // let qc = Q_C.init(Queue::new());

    // Fill demo segments for X/Y (5 kHz)
    let half_x = x.half_period_for_rate(5_000); // x is the stepper motor instance
    let half_y = y.half_period_for_rate(5_000); // y is the stepper motor instance
    motion::fill_test_segments(qx, half_x);
    motion::fill_test_segments(qy, half_y);

    // Feeder loop for X/Y (replace Timer with IRQ-paced in production)
    loop {
        if let Some(seg) = qx.dequeue() {
            x.start_segment_dma(seg).await;
            let est_ms = (seg.steps as u64 * 1000) / 5_000;
            Timer::after(Duration::from_millis(est_ms)).await;
        }
        if let Some(seg) = qy.dequeue() {
            y.start_segment_dma(seg).await;
            let est_ms = (seg.steps as u64 * 1000) / 5_000;
            Timer::after(Duration::from_millis(est_ms)).await;
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}
