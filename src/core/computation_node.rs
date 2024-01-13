// Copyright @yucwang 2021

pub trait ComputationNode {
    // Output string for a single computation node.
    fn to_string(&self) -> String;
}
