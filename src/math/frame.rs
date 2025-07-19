// Copyright @yucwang 2023

use crate::math::constants::{ Vector3f };

struct Frame {
    x: Vector3f,
    y: Vector3f,
    z: Vector3f
}

impl Default for Frame {
    fn default() -> Self {
        Frame {
            x: Vector3f::new(1.0, 0.0, 0.0),
            y: Vector3f::new(0.0, 1.0, 0.0),
            z: Vector3f::new(0.0, 0.0, 1.0)
        }
    }
}

impl Frame {
    fn new(new_x: Vector3f, new_y: Vector3f, new_z: Vector3f) -> Frame {
        Frame {
            x: new_x,
            y: new_y,
            z: new_z
        }
    }
    
    fn from_xy(new_x: Vector3f, new_y: Vector3f) -> Frame {
        Frame {
            x: new_x, 
            y: new_y,
            z: new_x.cross(&new_y)
        }
    }

    fn from_xz(new_x: Vector3f, new_z: Vector3f) -> Frame {
        Frame {
            x: new_x,
            y: new_x.cross(&new_z),
            z: new_z
        }
    }

    fn to_local(&self, v: Vector3f) -> Vector3f {
        Vector3f::new(v.dot(&self.x), v.dot(&self.y), v.dot(&self.z))
    }

    fn from_local(&self, v: Vector3f) -> Vector3f {
        v.x * self.x + v.y * self.y + v.z * self.z
    }
}
