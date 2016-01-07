extern crate nalgebra as na;
extern crate num;

use na::{Norm, FloatPnt, FloatVec};
use num::Zero;

struct Node<T, F> {
    parent: usize,
    position: T,
    growth: F,
}

pub struct SpaceColonization<T, F>
    where T: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    nodes: Vec<Node<T, F>>,
}

impl<T, F> SpaceColonization<T, F>
    where T: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    pub fn new() -> SpaceColonization<T, F> {
        SpaceColonization { nodes: Vec::new() }
    }

    pub fn add_root_node(&mut self, position: T) {
        self.add_node(position, None);
    }

    fn add_node(&mut self, position: T, parent: Option<usize>) {
        // NOTE: a root node has it's own index as parent
        let len = self.nodes.len();
        let parent = match parent {
            Some(p) => {
                assert!(p < len);
                p
            }
            None => len,
        };

        self.nodes.push(Node {
            parent: parent,
            position: position,
            growth: Zero::zero(),
        });
    }

    pub fn iter_segments<C>(&self, callback: &mut C)
        where C: FnMut(&T, &T)
    {
        for (i, node) in self.nodes.iter().enumerate() {
            if i != node.parent {
                callback(&node.position, &self.nodes[node.parent].position);
            }
        }
    }

    pub fn iterate(&mut self,
                   attraction_points: &mut [(T, bool)],
                   influence_radius_sq: f32,
                   move_distance: f32,
                   kill_distance_sq: f32)
                   -> usize {
        assert!(kill_distance_sq <= influence_radius_sq);

        // for each attraction_point, find the nearest node that it influences
        for ap in attraction_points.iter_mut() {
            let active = ap.1;
            if !active {
                continue;
            }

            // find the node nearest to the `ap` attraction point
            let mut nearest_node: Option<usize> = None;
            let mut nearest_distance_sq: f32 = influence_radius_sq;
            for (i, node) in self.nodes.iter().enumerate() {
                let dist_sq = node.position.sqdist(&ap.0);

                if dist_sq < kill_distance_sq {
                    // set attraction point inactive
                    ap.1 = false;
                    nearest_node = None;
                    break;
                }

                // is the attraction point within the radius of influence
                // and closer than the currently best
                if dist_sq < nearest_distance_sq {
                    nearest_distance_sq = dist_sq;
                    nearest_node = Some(i);
                }
            }

            if let Some(nearest_node_idx) = nearest_node {
                // update the force with the normalized vector towards the attraction point
                let v = (ap.0 - self.nodes[nearest_node_idx].position).normalize();
                self.nodes[nearest_node_idx].growth = self.nodes[nearest_node_idx].growth + v;
            }
        }

        // now create new nodes
        let mut new_nodes = 0;
        let len = self.nodes.len();
        for i in 0..len {
            let growth = self.nodes[i].growth;
            // update leaf node only if there is a force!
            if !growth.is_zero() {
                new_nodes += 1;
                let n = growth.normalize();
                let new_position = self.nodes[i].position + (n * move_distance);
                self.add_node(new_position, Some(i));
                // and reset growth attraction forces
                self.nodes[i].growth = Zero::zero();
            }
        }

        return new_nodes;
    }
}
