// Copyright @yucwang 2021

use std::sync::atomic::{AtomicU64, Ordering};

static NEXT_NODE_ID: AtomicU64 = AtomicU64::new(1);

/// Generate a unique default ID for a computation node.
pub fn generate_node_id(type_name: &str) -> String {
    let seq = NEXT_NODE_ID.fetch_add(1, Ordering::Relaxed);
    format!("{}_{}", type_name, seq)
}

pub trait ComputationNode {
    /// Return the unique identifier for this computation node.
    fn id(&self) -> &str;

    // Output string for a single computation node.
    fn to_string(&self) -> String;
}
