
use heapless::spsc::Queue;
use crate::hw::Segment;
use crate::AXIS_QUEUE_DEPTH;

pub fn fill_test_segments(q: &mut Queue<Segment, {AXIS_QUEUE_DEPTH}>, half_cycles: u32) {
    let fwd = Segment { half_period_cycles: half_cycles, steps: 10_000, dir_high: false };
    let rev = Segment { half_period_cycles: half_cycles, steps: 10_000, dir_high: true };
    let _ = q.enqueue(fwd);
    let _ = q.enqueue(rev);
}
