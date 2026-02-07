// Copyright 2020 TwoCookingMice

#![allow(dead_code)]

pub extern crate nalgebra as na;

pub mod core;
pub mod io;
pub mod integrators;
pub mod materials;
pub mod math;
pub mod emitters;
pub mod media;
pub mod renderers;
pub mod sensors;
pub mod shapes;
pub mod textures;
pub mod volumes;

use crate::core::scene_loader::load_scene_with_settings;
use crate::core::integrator::Integrator;
use crate::integrators::path::PathIntegrator;
use crate::integrators::raymarching::RaymarchingIntegrator;
use crate::math::bitmap::Bitmap;
use crate::renderers::simple::{Renderer, SimpleRenderer};

pub fn render_scene(
    scene_path: &str,
    spp_override: Option<u32>,
    max_depth_override: Option<u32>,
    seed: u64,
    camera_id: usize,
) -> Result<Bitmap, String> {
    let _ = env_logger::try_init();

    let load_result = load_scene_with_settings(scene_path)
        .map_err(|err| format!("failed to load scene: {:?}", err))?;

    let mut scene = load_result.scene;
    let spp = spp_override.or(load_result.samples_per_pixel).unwrap_or(1);
    let max_depth = max_depth_override.or(load_result.max_depth).unwrap_or(1);
    let integrator_name = load_result.integrator_type.as_deref().unwrap_or("path");
    let integrator: Box<dyn Integrator> = match integrator_name {
        "path" => Box::new(PathIntegrator::new(max_depth, spp)),
        "raymarching" => Box::new(RaymarchingIntegrator::new(max_depth, spp, None)),
        other => {
            eprintln!("Unsupported integrator '{}', falling back to path.", other);
            Box::new(PathIntegrator::new(max_depth, spp))
        }
    };

    let renderer: SimpleRenderer = SimpleRenderer::new(integrator, camera_id, seed);
    Ok(renderer.render(&mut scene))
}

#[cfg(feature = "python")]
mod python {
    use crate::core::scene::Scene;
    use crate::core::scene_loader::load_scene_with_settings;
    use crate::core::integrator::Integrator;
    use crate::integrators::path::PathIntegrator;
    use crate::renderers::simple::{Renderer, SimpleRenderer};
    use crate::math::constants::MatrixXF;
    use pyo3::exceptions::PyRuntimeError;
    use pyo3::prelude::*;
    use pyo3::types::PyDict;
    use nalgebra_py::matrix_to_numpy;
    use std::sync::Mutex;

    #[pyclass(unsendable)]
    struct PyScene {
        scene: Mutex<Scene>,
    }

    #[pymethods]
    impl PyScene {
        #[getter]
        fn data(&self, py: Python<'_>) -> PyResult<PyObject> {
            self.raw_data(py)
        }

        fn raw_data_keys(&self) -> Vec<String> {
            self.scene
                .lock()
                .expect("scene lock")
                .raw_data()
                .keys()
                .cloned()
                .collect()
        }

        fn raw_data(&self, py: Python<'_>) -> PyResult<PyObject> {
            let dict = PyDict::new(py);
            let scene = self.scene.lock().expect("scene lock");
            for (key, view) in scene.raw_data() {
                let len = view.rows * view.cols;
                let matrix = if len == 0 {
                    MatrixXF::zeros(view.rows, view.cols)
                } else {
                    let slice = unsafe { std::slice::from_raw_parts(view.ptr, len) };
                    MatrixXF::from_column_slice(view.rows, view.cols, slice)
                };
                dict.set_item(key, matrix_to_numpy(py, &matrix))?;
            }
            Ok(dict.to_object(py))
        }

        fn raw_data_item(&self, py: Python<'_>, key: &str) -> PyResult<Option<PyObject>> {
            let scene = self.scene.lock().expect("scene lock");
            let view = match scene.raw_data_view(key) {
                Some(view) => view,
                None => return Ok(None),
            };
            let len = view.rows * view.cols;
            let matrix = if len == 0 {
                MatrixXF::zeros(view.rows, view.cols)
            } else {
                let slice = unsafe { std::slice::from_raw_parts(view.ptr, len) };
                MatrixXF::from_column_slice(view.rows, view.cols, slice)
            };
            Ok(Some(matrix_to_numpy(py, &matrix)))
        }
    }

    #[pyfunction]
    fn load_scene(py: Python<'_>, scene_path: &str) -> PyResult<Py<PyScene>> {
        let load_result = load_scene_with_settings(scene_path)
            .map_err(|err| PyRuntimeError::new_err(format!("failed to load scene: {:?}", err)))?;
        Py::new(py, PyScene { scene: Mutex::new(load_result.scene) })
    }

    #[pyfunction]
    fn render(py: Python<'_>, scene: &PyScene, spp: u32, max_depth: u32) -> PyResult<PyObject> {
        let integrator: Box<dyn Integrator> = Box::new(PathIntegrator::new(max_depth, spp));
        let renderer: SimpleRenderer = SimpleRenderer::new(integrator, 0, 0);
        let mut scene = scene.scene.lock().expect("scene lock");
        let image = renderer.render(&mut scene);
        Ok(matrix_to_numpy(py, image.matrix()))
    }

    #[pymodule]
    fn dacquoise(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
        m.add_function(wrap_pyfunction!(render, m)?)?;
        m.add_function(wrap_pyfunction!(load_scene, m)?)?;
        Ok(())
    }
}
