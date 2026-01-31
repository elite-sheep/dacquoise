// Copyright @yucwang 2026

use crate::math::constants::Float;

pub struct LcgRng {
    state: u64,
}

impl LcgRng {
    pub fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    pub fn next_u32(&mut self) -> u32 {
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1);
        (self.state >> 32) as u32
    }

    pub fn next_f32(&mut self) -> Float {
        (self.next_u32() as Float) / (u32::MAX as Float)
    }
}
