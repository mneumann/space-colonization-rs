extern crate nalgebra as na;
extern crate num;

use na::{Norm, FloatPnt, FloatVec};
use num::Zero;
use std::cmp;

struct Node<T, F> {
    parent: usize,
    position: T,

    // The active_lifetime of a node is reduced every
    // time it is found being the closest node to some
    // attraction point. If it hits 0, the node
    // is no longer used.
    active_lifetime: i32,
    // The inactive_lifetime is reduce whenever a node
    // is not being found as the closest towards an
    // attraction point.
    inactive_lifetime: i32,

    growth: F,
    growth_count: usize,
}

pub struct SpaceColonization<T, F>
    where T: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    nodes: Vec<Node<T, F>>,
    default_active_lifetime: i32,
    default_inactive_lifetime: i32,
}

impl<T, F> SpaceColonization<T, F>
    where T: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    pub fn new(default_active_lifetime: i32,
               default_inactive_lifetime: i32)
               -> SpaceColonization<T, F> {
        SpaceColonization {
            nodes: Vec::new(),
            default_active_lifetime: default_active_lifetime,
            default_inactive_lifetime: default_inactive_lifetime,
        }
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
            active_lifetime: self.default_active_lifetime,
            inactive_lifetime: self.default_inactive_lifetime,
            growth: Zero::zero(),
            growth_count: 0,
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
                   influence_radius: f32,
                   move_distance: f32,
                   kill_distance: f32,
                   use_last_n_nodes: Option<usize>)
                   -> usize {
        let kill_distance_sq = kill_distance * kill_distance;
        let influence_radius_sq = influence_radius * influence_radius;
        assert!(kill_distance_sq <= influence_radius_sq);

        let num_nodes = self.nodes.len();
        let use_last_nodes: usize = cmp::min(num_nodes, use_last_n_nodes.unwrap_or(num_nodes));
        let start_index = num_nodes - use_last_nodes;

        {
            let nodes = &mut self.nodes[start_index..];

            // for each attraction_point, find the nearest node that it influences
            for ap in attraction_points.iter_mut() {
                let active = ap.1;
                if !active {
                    continue;
                }

                // find the node nearest to the `ap` attraction point
                let mut nearest_node: Option<&mut Node<_, _>> = None;
                let mut nearest_distance_sq: f32 = influence_radius_sq;
                for node in nodes.iter_mut() {
                    if node.inactive_lifetime <= 0 || node.active_lifetime <= 0 {
                        // The node has become inactive
                        continue;
                    }

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
                        nearest_node = Some(node);
                    }
                }

                if let Some(node) = nearest_node {
                    // update the force with the normalized vector towards the attraction point
                    let v = (ap.0 - node.position).normalize();
                    node.growth = node.growth + v;
                    node.growth_count += 1;
                }
            }
        }


        // now create new nodes
        for i in start_index..num_nodes {
            if self.nodes[i].growth_count > 0 {
                let d = self.nodes[i].growth.normalize() * move_distance;
                let new_position = self.nodes[i].position + d;
                self.add_node(new_position, Some(i));

                self.nodes[i].active_lifetime -= 1;
                self.nodes[i].inactive_lifetime = self.default_inactive_lifetime; // XXX
            } else {
                self.nodes[i].inactive_lifetime -= 1;
            }
            // and reset growth attraction forces
            self.nodes[i].growth = Zero::zero();
            self.nodes[i].growth_count = 0;
        }

        // Note that nodes can oscillate, between two attraction points, so
        // it's better to stop after a certain number of iterations
        return self.nodes.len() - num_nodes;
    }
}
