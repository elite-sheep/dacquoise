use std::fs;
use std::path::Path;

use wavefront_obj::{obj, ParseError};

#[derive(Debug)]
pub enum ObjLoadError {
    Io(std::io::Error),
    Parse(ParseError),
}

impl From<std::io::Error> for ObjLoadError {
    fn from(err: std::io::Error) -> Self {
        ObjLoadError::Io(err)
    }
}

impl From<ParseError> for ObjLoadError {
    fn from(err: ParseError) -> Self {
        ObjLoadError::Parse(err)
    }
}

pub fn load_obj_from_str<S: AsRef<str>>(input: S) -> Result<obj::ObjSet, ParseError> {
    obj::parse(input)
}

pub fn load_obj_from_file<P: AsRef<Path>>(path: P) -> Result<obj::ObjSet, ObjLoadError> {
    let data = fs::read_to_string(path)?;
    let obj_set = obj::parse(data)?;
    Ok(obj_set)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_obj_from_str_basic() {
        let input = "\
v 0.0 0.0 0.0
v 1.0 0.0 0.0
v 0.0 1.0 0.0
f 1 2 3
";
        let obj_set = load_obj_from_str(input).expect("failed to parse obj");
        assert_eq!(obj_set.objects.len(), 1);
        let object = &obj_set.objects[0];
        assert_eq!(object.vertices.len(), 3);
        assert_eq!(object.geometry.len(), 1);
    }
}
