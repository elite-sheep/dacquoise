/* Copyright 2020 @TwoCookingMice */

use crate::math::constants::{ Float, UInt };

use openexr::frame_buffer::{ FrameBuffer, FrameBufferMut };
use openexr::header::Header;
use openexr::input::InputFile;
use openexr::{ PixelType, ScanlineOutputFile };

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

// Write EXR Image to file
pub fn write_exr_to_file(image: &std::vec::Vec<(Float, Float, Float)>, 
                         width: UInt,
                         height: UInt,
                         file_path: &str) {
    log::info!("Starting writing openexr images: {}.", file_path);

    let mut file = std::fs::File::create(file_path).unwrap();
    let mut output_file = ScanlineOutputFile::new(
        &mut file,
        Header::new()
        .set_resolution(width, height)
        .add_channel("R", PixelType::FLOAT)
        .add_channel("G", PixelType::FLOAT)
        .add_channel("B", PixelType::FLOAT)).unwrap();

    let mut frame_buffer = FrameBuffer::new(width, height);
    frame_buffer.insert_channels(&["R", "G", "B"], &image);
    output_file.write_pixels(&frame_buffer).unwrap();
}
