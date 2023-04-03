/* Copyright 2020 @TwoCookingMice */

use crate::math::constants::Float;

use exr::prelude::*;

// Read EXR Image from file
pub fn read_exr_from_file(file_path: &str) {
    log::info!("Starting reading OpenEXR image from: {}.", file_path);

    // let mut file = std::fs::File::open(file_path).unwrap();
    // let mut input_file = InputFile::new(&mut file).unwrap();

    // let (width, height) = input_file.header().data_dimensions();

    // let mut pixel_array = vec![(0.0f32, 0.0f32, 0.0f32); (width * height) as usize];

    // {
    //     let mut frame_buffer = FrameBufferMut::new(width, 
    //                                                height); 
    //     frame_buffer.insert_channels(&[("R", 0.0), ("G", 0.0), ("B", 0.0)], &mut pixel_array);
    //     input_file.read_pixels(&mut frame_buffer).unwrap();
    // }

    // log::info!("OpenEXR loaded, width = {}, height = {}.", width, height);
}

// Write EXR Image to file
pub fn write_exr_to_file(image: &std::vec::Vec<(Float, Float, Float)>, 
                         width: usize,
                         height: usize,
                         file_path: &str) {
    log::info!("Starting writing openexr images: {}.", file_path);

    let write_result = write_rgb_file(file_path, width, height, |x,y| {
        (
            image[y*width+x].0,
            image[y*width+x].1,
            image[y*width+x].2
        )
    });
    match write_result {
        Ok(()) => println!("EXR written to: {}.", file_path),
        Err(e) => println!("EXR written error: {}.", e.to_string())
    }
}
