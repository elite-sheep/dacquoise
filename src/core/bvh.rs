// Copyright @yucwang 2026

use crate::math::aabb::AABB;
use crate::math::constants::{Float, Vector3f};
use crate::math::ray::Ray3f;

const SAH_BUCKETS: usize = 12;

#[derive(Clone)]
struct BVHNode {
    bounds: AABB,
    left: Option<usize>,
    right: Option<usize>,
    start: usize,
    count: usize,
}

impl BVHNode {
    fn leaf(bounds: AABB, start: usize, count: usize) -> Self {
        Self { bounds, left: None, right: None, start, count }
    }

    fn interior(bounds: AABB, left: usize, right: usize) -> Self {
        Self { bounds, left: Some(left), right: Some(right), start: 0, count: 0 }
    }

    fn is_leaf(&self) -> bool {
        self.count > 0
    }
}

pub struct BVH {
    nodes: Vec<BVHNode>,
    indices: Vec<usize>,
    prim_bounds: Vec<AABB>,
    prim_centroids: Vec<Vector3f>,
    max_leaf_size: usize,
}

impl BVH {
    pub fn new(prim_bounds: Vec<AABB>, prim_centroids: Vec<Vector3f>) -> Self {
        Self::with_max_leaf_size(prim_bounds, prim_centroids, 4)
    }

    pub fn with_max_leaf_size(
        prim_bounds: Vec<AABB>,
        prim_centroids: Vec<Vector3f>,
        max_leaf_size: usize,
    ) -> Self {
        // BVH stores only primitive bounds/centroids; intersection is delegated via callbacks.
        let mut bvh = Self {
            indices: (0..prim_bounds.len()).collect(),
            nodes: Vec::new(),
            prim_bounds,
            prim_centroids,
            max_leaf_size: max_leaf_size.max(1),
        };

        if !bvh.indices.is_empty() {
            let (bounds, centroid_bounds) = bvh.compute_bounds(0, bvh.indices.len());
            bvh.build(0, bvh.indices.len(), bounds, centroid_bounds);
        }

        bvh
    }

    pub fn ray_intersection<F, T>(&self, ray: &Ray3f, mut hit_fn: F) -> Option<(usize, T)>
    where
        F: FnMut(usize, &Ray3f) -> Option<(T, Float)>,
    {
        // Traversal returns the closest hit reported by the callback.
        if self.nodes.is_empty() {
            return None;
        }

        let mut closest: Option<(usize, T, Float)> = None;
        let mut closest_t = std::f32::MAX;
        let mut stack = vec![0usize];

        while let Some(node_idx) = stack.pop() {
            let node = &self.nodes[node_idx];
            if !node.bounds.ray_intersect(&ray) {
                continue;
            }

            if node.is_leaf() {
                for i in 0..node.count {
                    let prim_idx = self.indices[node.start + i];
                    if let Some((hit, t)) = hit_fn(prim_idx, ray) {
                        if t < closest_t {
                            closest_t = t;
                            closest = Some((prim_idx, hit, t));
                        }
                    }
                }
            } else {
                // Depth-first traversal with an explicit stack.
                if let Some(left) = node.left {
                    stack.push(left);
                }
                if let Some(right) = node.right {
                    stack.push(right);
                }
            }
        }

        closest.map(|(idx, hit, _)| (idx, hit))
    }

    pub fn ray_intersection_t<F>(&self, ray: &Ray3f, mut hit_fn: F) -> bool
    where
        F: FnMut(usize, &Ray3f) -> bool,
    {
        // Early-out traversal for shadow rays.
        if self.nodes.is_empty() {
            return false;
        }

        let mut stack = vec![0usize];
        while let Some(node_idx) = stack.pop() {
            let node = &self.nodes[node_idx];
            if !node.bounds.ray_intersect(&ray) {
                continue;
            }
            if node.is_leaf() {
                for i in 0..node.count {
                    let prim_idx = self.indices[node.start + i];
                    if hit_fn(prim_idx, ray) {
                        return true;
                    }
                }
            } else {
                if let Some(left) = node.left {
                    stack.push(left);
                }
                if let Some(right) = node.right {
                    stack.push(right);
                }
            }
        }

        false
    }

    fn build(&mut self, start: usize, end: usize, bounds: AABB, centroid_bounds: AABB) -> usize {
        let count = end - start;
        if count <= self.max_leaf_size {
            // Small enough: create a leaf.
            let node_idx = self.nodes.len();
            self.nodes.push(BVHNode::leaf(bounds, start, count));
            return node_idx;
        }

        let axis = centroid_bounds.max_extent() as usize;
        let axis_min = centroid_bounds.p_min[axis];
        let axis_max = centroid_bounds.p_max[axis];
        if (axis_max - axis_min).abs() < 1e-6 {
            // Degenerate centroid bounds: fall back to leaf.
            let node_idx = self.nodes.len();
            self.nodes.push(BVHNode::leaf(bounds, start, count));
            return node_idx;
        }

        // SAH with fixed buckets along the split axis.
        let mut buckets = vec![(0usize, AABB::default()); SAH_BUCKETS];
        for i in start..end {
            let idx = self.indices[i];
            let c = self.prim_centroids[idx][axis];
            let mut b = ((c - axis_min) / (axis_max - axis_min) * SAH_BUCKETS as Float) as usize;
            if b >= SAH_BUCKETS {
                b = SAH_BUCKETS - 1;
            }
            buckets[b].0 += 1;
            let mut bnd = buckets[b].1;
            bnd.expand_by_aabb(&self.prim_bounds[idx]);
            buckets[b].1 = bnd;
        }

        let mut cost = [0.0f32; SAH_BUCKETS - 1];
        for i in 0..(SAH_BUCKETS - 1) {
            let mut b0 = AABB::default();
            let mut b1 = AABB::default();
            let mut count0 = 0usize;
            let mut count1 = 0usize;
            for b in 0..=i {
                count0 += buckets[b].0;
                b0.expand_by_aabb(&buckets[b].1);
            }
            for b in (i + 1)..SAH_BUCKETS {
                count1 += buckets[b].0;
                b1.expand_by_aabb(&buckets[b].1);
            }
            let area = bounds.surface_area().max(1e-6);
            let cost0 = if count0 > 0 {
                (count0 as Float) * b0.surface_area()
            } else {
                0.0
            };
            let cost1 = if count1 > 0 {
                (count1 as Float) * b1.surface_area()
            } else {
                0.0
            };
            cost[i] = 1.0 + (cost0 + cost1) / area;
        }

        let mut min_cost = cost[0];
        let mut min_split = 0usize;
        for i in 1..cost.len() {
            if cost[i] < min_cost {
                min_cost = cost[i];
                min_split = i;
            }
        }

        // If SAH says leaf is cheaper, stop splitting.
        let leaf_cost = count as Float;
        if min_cost >= leaf_cost {
            let node_idx = self.nodes.len();
            self.nodes.push(BVHNode::leaf(bounds, start, count));
            return node_idx;
        }

        // Partition indices in-place by bucket.
        let mut mid = start;
        let mut i = start;
        while i < end {
            let idx = self.indices[i];
            let c = self.prim_centroids[idx][axis];
            let mut b = ((c - axis_min) / (axis_max - axis_min) * SAH_BUCKETS as Float) as usize;
            if b >= SAH_BUCKETS {
                b = SAH_BUCKETS - 1;
            }
            if b <= min_split {
                self.indices.swap(i, mid);
                mid += 1;
            }
            i += 1;
        }

        if mid == start || mid == end {
            // Partition failed: create a leaf.
            let node_idx = self.nodes.len();
            self.nodes.push(BVHNode::leaf(bounds, start, count));
            return node_idx;
        }

        // Build child nodes and stitch them into an interior node.
        let (left_bounds, left_centroids) = self.compute_bounds(start, mid);
        let (right_bounds, right_centroids) = self.compute_bounds(mid, end);
        let node_idx = self.nodes.len();
        self.nodes.push(BVHNode::leaf(bounds, 0, 0));
        let left = self.build(start, mid, left_bounds, left_centroids);
        let right = self.build(mid, end, right_bounds, right_centroids);
        self.nodes[node_idx] = BVHNode::interior(bounds, left, right);
        node_idx
    }

    fn compute_bounds(&self, start: usize, end: usize) -> (AABB, AABB) {
        let mut bounds = AABB::default();
        let mut centroid_bounds = AABB::default();
        for i in start..end {
            let idx = self.indices[i];
            bounds.expand_by_aabb(&self.prim_bounds[idx]);
            centroid_bounds.expand_by_point(&self.prim_centroids[idx]);
        }
        (bounds, centroid_bounds)
    }
}

#[cfg(test)]
mod tests {
    use super::BVH;
    use crate::core::shape::Shape;
    use crate::math::constants::{Float, Vector3f};
    use crate::math::ray::Ray3f;
    use crate::shapes::triangle::Triangle;

    fn build_triangles() -> Vec<Triangle> {
        let mut tris = Vec::new();
        for i in 0..8 {
            let x = i as Float * 2.0;
            let p0 = Vector3f::new(x, 0.0, 0.0);
            let p1 = Vector3f::new(x + 0.5, 0.0, 0.0);
            let p2 = Vector3f::new(x, 0.5, 0.0);
            tris.push(Triangle::new(p0, p1, p2));
        }
        tris
    }

    #[test]
    fn test_bvh_vs_naive_triangles() {
        let triangles = build_triangles();
        let mut prim_bounds = Vec::with_capacity(triangles.len());
        let mut prim_centroids = Vec::with_capacity(triangles.len());
        for tri in &triangles {
            let b = tri.bounding_box();
            prim_centroids.push(b.center());
            prim_bounds.push(b);
        }

        let bvh = BVH::new(prim_bounds, prim_centroids);

        for (i, _tri) in triangles.iter().enumerate() {
            let origin = Vector3f::new(i as Float * 2.0 + 0.1, 0.1, 1.0);
            let ray = Ray3f::new(origin, Vector3f::new(0.0, 0.0, -1.0), None, None);

            let bvh_hit: Option<(usize, Float)> = bvh.ray_intersection(&ray, |prim_idx, ray| {
                if let Some(h) = triangles[prim_idx].ray_intersection(ray) {
                    let t = h.t();
                    Some((t, t))
                } else {
                    None
                }
            });

            let mut naive_t: Option<Float> = None;
            for tri in &triangles {
                if let Some(hit) = tri.ray_intersection(&ray) {
                    let t = hit.t();
                    if naive_t.map_or(true, |cur| t < cur) {
                        naive_t = Some(t);
                    }
                }
            }

            assert!(bvh_hit.is_some(), "BVH miss for ray {}", i);
            let bvh_t = bvh_hit.unwrap().1;
            let naive_t = naive_t.expect("Naive miss");
            assert!((bvh_t - naive_t).abs() < 1e-5);
        }

        let miss_ray = Ray3f::new(Vector3f::new(100.0, 100.0, 1.0), Vector3f::new(0.0, 0.0, -1.0), None, None);
        let bvh_miss: Option<(usize, Float)> = bvh.ray_intersection(&miss_ray, |prim_idx, ray| {
            if let Some(h) = triangles[prim_idx].ray_intersection(ray) {
                let t = h.t();
                Some((t, t))
            } else {
                None
            }
        });
        assert!(bvh_miss.is_none());
    }
}
