use std::fs;
use std::path::Path;

use wavefront_obj::{obj, ParseError};
use std::fmt;

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

impl fmt::Display for ObjLoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ObjLoadError::Io(err) => write!(f, "io error: {}", err),
            ObjLoadError::Parse(err) => write!(f, "parse error: {}", err),
        }
    }
}

impl std::error::Error for ObjLoadError {}

pub fn load_obj_from_str<S: AsRef<str>>(input: S) -> Result<obj::ObjSet, ParseError> {
    let triangulated = triangulate_faces(input.as_ref());
    obj::parse(triangulated)
}

pub fn load_obj_from_file<P: AsRef<Path>>(path: P) -> Result<obj::ObjSet, ObjLoadError> {
    let data = fs::read_to_string(path)?;
    let obj_set = load_obj_from_str(data)?;
    Ok(obj_set)
}

fn triangulate_faces(input: &str) -> String {
    let mut out = String::with_capacity(input.len() + input.len() / 4);
    for line in input.lines() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("f ") || trimmed.starts_with("f\t") {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() > 4 {
                let base = parts[1];
                for i in 2..(parts.len() - 1) {
                    out.push_str("f ");
                    out.push_str(base);
                    out.push(' ');
                    out.push_str(parts[i]);
                    out.push(' ');
                    out.push_str(parts[i + 1]);
                    out.push('\n');
                }
                continue;
            }
        }
        out.push_str(line);
        out.push('\n');
    }
    out
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
