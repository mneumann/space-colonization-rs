extern crate nalgebra as na;
extern crate num;

use na::{Norm, FloatPnt, FloatVec};
use num::Zero;
use std::cmp;

/// Wraps a square distance.
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub struct SqDist(pub f32);

impl SqDist {
    pub fn from_dist(d: f32) -> SqDist {
        SqDist(d.powi(2))
    }
}

/// What to do when a node `connects` with an attrator.
#[derive(Debug, Copy, Clone)]
pub enum ConnectAction {
    KillAttractor,
    DisableFor {
        iterations: u32,
    },
}

#[derive(Debug, Copy, Clone)]
pub struct Attractor<P, I> {
    /// The radius^2 within which it can influence a Node.
    attract_radius_sq: SqDist,

    /// If there is a node closer than the square root of
    /// this radius, the information is exchanged with the
    /// node and the ```connect_action``` is performed.
    /// This can be for example: kill the attractor,
    /// or disable it for a while.
    connect_radius_sq: SqDist,

    /// The strenght with which it influences a Node.
    strength: f32,

    /// The position of the attractor.
    pub position: P,

    /// The attractor carries a bit of information.
    /// When a node comes closer than ```connect_radius```
    /// this bit of information is exchanged.
    information: I,

    /// Action performed when a node comes closer
    /// than ```connect_radius```.
    connect_action: ConnectAction,
}

struct Node<P, F> {
    /// Index of the direct parent.
    parent: usize,

    /// Number of nodes between this node and the root node.
    length: usize,

    /// Index of the root node this node is associated with.
    root: usize,

    /// The node's coordinate position.
    position: P,

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
    growth_count: u32,
}
impl<P, F> Node<P, F> {
    fn send_information(&mut self, _information: ()) {}
}

pub struct SpaceColonization<P, F>
    where P: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    nodes: Vec<Node<P, F>>,
    attractors: Vec<Attractor<P, ()>>,
    default_active_lifetime: i32,
    default_inactive_lifetime: i32,
    default_attract_radius_sq: SqDist,
    default_connect_radius_sq: SqDist,
}

impl<P, F> SpaceColonization<P, F>
    where P: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    pub fn new(default_active_lifetime: i32,
               default_inactive_lifetime: i32,
               default_attract_radius_sq: SqDist,
               default_connect_radius_sq: SqDist)
               -> SpaceColonization<P, F> {
        SpaceColonization {
            nodes: Vec::new(),
            attractors: Vec::new(),
            default_active_lifetime: default_active_lifetime,
            default_inactive_lifetime: default_inactive_lifetime,
            default_attract_radius_sq: default_attract_radius_sq,
            default_connect_radius_sq: default_connect_radius_sq,
        }
    }

    pub fn attractors(&self) -> &[Attractor<P, ()>] {
        &self.attractors
    }

    pub fn add_attractor(&mut self, position: P) {
        self.attractors.push(Attractor {
            attract_radius_sq: self.default_attract_radius_sq,
            connect_radius_sq: self.default_connect_radius_sq,
            strength: 1.0,
            position: position,
            information: (),
            connect_action: ConnectAction::KillAttractor,
        });
    }

    pub fn add_root_node(&mut self, position: P) {
        self.add_node(position, None);
    }

    fn add_node(&mut self, position: P, parent: Option<usize>) {
        // NOTE: a root node has it's own index as parent and root.
        let len = self.nodes.len();
        let (parent, root, length) = match parent {
            Some(p) => {
                assert!(p < len);
                (p, self.nodes[p].root, self.nodes[p].length + 1)
            }
            None => (len, len, 0),
        };

        self.nodes.push(Node {
            parent: parent,
            root: root,
            length: length,
            position: position,
            active_lifetime: self.default_active_lifetime,
            inactive_lifetime: self.default_inactive_lifetime,
            growth: Zero::zero(),
            growth_count: 0,
        });
    }

    pub fn iter_segments<C>(&self, callback: &mut C)
        where C: FnMut(&P, &P)
    {
        for (i, node) in self.nodes.iter().enumerate() {
            if i != node.parent {
                callback(&node.position, &self.nodes[node.parent].position);
            }
        }
    }

    pub fn iterate(&mut self, move_distance: f32, use_last_n_nodes: Option<usize>) -> usize {
        let num_nodes = self.nodes.len();
        let use_last_nodes: usize = cmp::min(num_nodes, use_last_n_nodes.unwrap_or(num_nodes));
        let start_index = num_nodes - use_last_nodes;

        {
            let nodes = &mut self.nodes[start_index..];

            // for each attraction_point, find the nearest node that it influences
            let mut ap_idx = 0;
            'outer: while ap_idx < self.attractors.len() {
                let ap = self.attractors[ap_idx];

                // find the node nearest to the `ap` attraction point
                let mut nearest_node: Option<&mut Node<_, _>> = None;
                let mut nearest_distance_sq = ap.attract_radius_sq;
                let mut connect_node: Option<&mut Node<_, _>> = None;
                for node in nodes.iter_mut() {
                    if node.inactive_lifetime <= 0 || node.active_lifetime <= 0 {
                        // The node has become inactive
                        continue;
                    }

                    let dist_sq = SqDist(node.position.sqdist(&ap.position));

                    if dist_sq < ap.connect_radius_sq {
                        // This node is within the connect radius of a node.
                        // XXX: There might be a closer node, but we use
                        // the first we find.
                        connect_node = Some(node);
                        // outside the node loop, we perform some action
                        break;
                    } else if dist_sq < nearest_distance_sq {
                        // ```node``` is within the influence of the attraction point,
                        // and it's closer than the currently closest node.
                        nearest_distance_sq = dist_sq;
                        nearest_node = Some(node);
                    }
                }

                if let Some(node) = connect_node {
                    // TODO: exchange information
                    node.send_information(ap.information);
                    match ap.connect_action {
                        ConnectAction::KillAttractor => {
                            // remove attraction point
                            self.attractors.swap_remove(ap_idx);
                            // and continue with "next" (without increasing ap_idx)
                            continue 'outer;
                        }
                        ConnectAction::DisableFor {iterations: _} => {
                            unimplemented!();
                        }
                    }
                } else if let Some(node) = nearest_node {
                    // update the force with the normalized vector towards the attraction point
                    let v = (ap.position - node.position).normalize() * ap.strength;
                    node.growth = node.growth + v;
                    node.growth_count += 1;
                }

                // go to next attractor point
                ap_idx += 1;
            }
        }

        // now create new nodes
        for i in start_index..num_nodes {
            let growth_count = self.nodes[i].growth_count;
            if growth_count > 0 {
                let growth_factor = 1.0; //((growth_count + 1) as f32).ln();
                let d = self.nodes[i].growth.normalize() * move_distance * growth_factor;
                let new_position = self.nodes[i].position + d;
                self.add_node(new_position, Some(i));

                self.nodes[i].active_lifetime -= 1;
                self.nodes[i].inactive_lifetime = self.default_inactive_lifetime; // XXX

                // and reset growth attraction forces
                self.nodes[i].growth = Zero::zero();
                self.nodes[i].growth_count = 0;
            } else {
                self.nodes[i].inactive_lifetime -= 1;
            }
        }

        // Note that nodes can oscillate, between two attraction points, so
        // it's better to stop after a certain number of iterations
        return self.nodes.len() - num_nodes;
    }
}
