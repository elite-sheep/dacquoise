// Copyright 2020 TwoCookingMice

pub extern crate nalgebra as na;

mod core;
mod io;
mod math;
mod renderers;

use self::math::spectrum::RGBSpectrum;
use self::renderers::simple::{ SimpleRenderer, Renderer };

use std::env;

fn main() {
    env::set_var("RUST_LOG", "info");
    env_logger::init();

    let colors: [RGBSpectrum; 4] = [RGBSpectrum::new(0.5, 0.5, 0.0),
                                    RGBSpectrum::new(0.5, 0.0, 0.5),
                                    RGBSpectrum::new(0.0, 0.5, 0.5),
                                    RGBSpectrum::new(0.5, 0.5, 0.5)];

    let renderer: SimpleRenderer = SimpleRenderer::new(colors);
    renderer.render();
}
