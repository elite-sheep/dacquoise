// Copyright 2020 TwoCookingMice

use openexr::frame_buffer::FrameBufferMut;
use openexr::input::InputFile;

// Read EXR Image from file
pub fn read_exr_from_file(file_path: &str) {
    log::info!("Starting reading OpenEXR image from: {}.", file_path);

    let mut file = std::fs::File::open(file_path).unwrap();
    let mut input_file = InputFile::new(&mut file).unwrap();

    let (width, height) = input_file.header().data_dimensions();

    let mut pixel_array = vec![(0.0f32, 0.0f32, 0.0f32); (width * height) as usize];

    {
        let (origin_x, origin_y) = input_file.header().data_origin();

        let mut frame_buffer = FrameBufferMut::new_with_origin(origin_x, 
                                                               origin_y, 
                                                               width, 
                                                               height); 
        frame_buffer.insert_channels(&[("R", 0.0), ("G", 0.0), ("B", 0.0)], &mut pixel_array);
        input_file.read_pixels(&mut frame_buffer).unwrap();
    }

    log::info!("OpenEXR loaded, width = {}, height = {}.", width, height);
}
