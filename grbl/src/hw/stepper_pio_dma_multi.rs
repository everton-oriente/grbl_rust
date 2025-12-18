
//! Generic PIO + DMA stepper driver for RP2350 (any PIO instance, any SM index).
//! STEP via PIO side-set; DIR/EN via GPIO; DMA feeds (half_period, steps) pairs; IRQ0 at segment end.
// Look at the pio_dma example in embassy github in https://github.com/embassy-rs/embassy/blob/main/examples/rp235x/src/bin/pio_dma.rs
use embassy_rp::dma::{Channel, Transfer};
use embassy_rp::gpio::Output;
use embassy_rp::pio::{Config as PioConfig, Pio, ShiftConfig, ShiftDirection};
use embassy_rp::pio::program::pio_asm;
use fixed_macro::types::U56F8;
use fixed::traits::ToFixed;
use defmt::info;


#[derive(Clone, Copy)]
pub struct Segment {
    pub half_period_cycles: u32,
    pub steps: u32,
    pub dir_high: bool,
}

pub struct Stepper<'d, P: Instance, const SM: usize> {
    sm: StateMachine<'d, P, SM>,
    common: Common<'d, P>,
    dma_ch: Channel<'d>,
    dir: Output<'d>,
    en: Output<'d>,
    pio_clk_hz: u32,
    // staging buffer for (half_period, steps)
    buf: [u32; 2],
}

impl<'d, P: Instance, const SM: usize> Stepper<'d, P, SM> {
    pub fn new(
        mut pio: Pio<'d, P>,
        common: Common<'d, P>,
        step_pin: u8,
        dma_ch: Channel<'d>,
        dir: Output<'d>,
        en: Output<'d>,
        pio_clk_hz: u32,
    ) -> Self {

/*
        let prg = pio_asm!(
        ".origin 0",
        "set pindirs,1",
        ".wrap_target",
        "set y,7",
        "loop:",
        "out x,4",
        "in x,4",
        "jmp y--, loop",
        ".wrap",
    );

        let mut cfg = Config::default();
        cfg.use_program(&common.load_program(&prg.program), &[]);
        cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(10_000)).to_fixed();
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Left,
        };
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Right,
        };

        sm.set_config(&cfg);
        sm.set_enable(true);
*/
        // Assemble PIO program: side-set 1 bit (STEP), pull half_period (OSR→Y), pull steps (OSR→X), pulse, delay, IRQ0 when done.
        let prg = pio_asm!(
        ".origin 0",
        "set pindirs,1",
        ".wrap_target",
        "set y,7",
        "loop:",
        "out x,4",
        "in x,4",
        "jmp y--, loop",
        ".wrap",
    );
        info!("Program assembled.");
        let mut cfg = PioConfig::default();
        cfg.use_program(&common.load_program(&prg.program), &[]);
        cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(10_000)).to_fixed();
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Left,
        };
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Right,
        };

        info!("Configuration of SM finished. Configuring SM.");
        sm.set_config(&cfg);
        info!("SM configured. Enabling SM.");
        sm.set_enable(true);
        info!("SM enabled.");

        Self { sm, common, dma_ch, dir, en, pio_clk_hz, buf: [0; 2] }
    }

    #[inline]
    pub fn half_period_for_rate(&self, step_rate_hz: u32) -> u32 {
        let half = self.pio_clk_hz / (2 * step_rate_hz);
        if half < 2 { 2 } else { half }
    }

    /// Start one segment by DMA-pushing (half_period, steps) into the PIO TX FIFO.
    pub async fn start_segment_dma(&mut self, seg: Segment) {
        if seg.dir_high { self.dir.set_high(); } else { self.dir.set_low(); }
        self.buf[0] = seg.half_period_cycles;
        self.buf[1] = seg.steps;
        let mut t = Transfer::new_write(
            &mut self.dma_ch,
            &self.buf,
            self.sm.tx().dma_write_target(),
            DmaConfig::default().data_request(self.sm.tx().dma_request()),
        );
        t.start();
        t.wait().await;
    }

    pub fn clear_irq0(&mut self) {
        self.common.clear_irq(IrqLevel::Irq0);
    }
}
