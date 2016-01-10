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
    /// The square distance within which it can influence a Node.
    attract_dist: SqDist,

    /// If there is a node closer than the square root of
    /// this distance, the information is exchanged with the
    /// node and the ```connect_action``` is performed.
    /// This can be for example: kill the attractor,
    /// or disable it for a while.
    connect_dist: SqDist,

    /// The strenght with which it influences a Node.
    strength: f32,

    /// The position of the attractor.
    position: P,

    /// The attractor carries a bit of information.
    /// When a node comes closer than ```connect_radius```
    /// this bit of information is exchanged.
    information: I,

    /// Action performed when a node comes closer
    /// than ```connect_radius```.
    connect_action: ConnectAction,
}

#[derive(Debug, Copy, Clone)]
struct NodeIdx(u32, u32);

struct Node<P, F> {
    /// Index of the direct parent.
    parent: NodeIdx,

    /// Index of the root node this node is associated with.
    root: NodeIdx,

    /// Number of nodes between this node and the root node.
    length: usize,

    /// The node's coordinate position.
    position: P,

    growth: F,
    growth_count: u32,
}

impl<P, F> Node<P, F> {
    fn send_information(&mut self, _information: ()) {}

    fn is_root(&self) -> bool {
        // self.root == self.parent
        self.length == 0
    }

    fn is_active(&self) -> bool {
        true
    }
}

pub struct SpaceColonization<P, F>
    where P: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    nodes: Vec<Node<P, F>>,
    attractors: Vec<Attractor<P, ()>>,
    default_attract_dist: SqDist,
    default_connect_dist: SqDist,
}

impl<P, F> SpaceColonization<P, F>
    where P: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    pub fn new(default_attract_dist: SqDist,
               default_connect_dist: SqDist)
               -> SpaceColonization<P, F> {
        SpaceColonization {
            nodes: Vec::new(),
            attractors: Vec::new(),
            default_attract_dist: default_attract_dist,
            default_connect_dist: default_connect_dist,
        }
    }

    pub fn add_attractor(&mut self, position: P) {
        self.attractors.push(Attractor {
            attract_dist: self.default_attract_dist,
            connect_dist: self.default_connect_dist,
            strength: 1.0,
            position: position,
            information: (),
            connect_action: ConnectAction::KillAttractor,
        });
    }

    pub fn add_root_node(&mut self, position: P) {
        self.add_node(position, None);
    }

    fn get_node(&self, node_idx: NodeIdx) -> Option<&Node<P, F>> {
        self.nodes.get(node_idx.0 as usize)
    }

    fn add_node(&mut self, position: P, parent: Option<NodeIdx>) {
        let (parent, root, length) = match parent {
            Some(p) => {
                let parent_node = self.get_node(p).unwrap();
                (p, parent_node.root, parent_node.length + 1)
            }
            None => {
                // A root node has it's own index as parent and root.
                let len = self.nodes.len();
                (NodeIdx(len as u32, 0), NodeIdx(len as u32, 0), 0)
            }
        };

        self.nodes.push(Node {
            parent: parent,
            root: root,
            length: length,
            position: position,
            growth: Zero::zero(),
            growth_count: 0,
        });
    }

    pub fn visit_node_segments<V>(&self, visitor: &mut V)
        where V: FnMut(&P, &P)
    {
        for node in self.nodes.iter() {
            if !node.is_root() {
                visitor(&node.position,
                        &self.get_node(node.parent).unwrap().position);
            }
        }
    }

    pub fn visit_attractor_points<V>(&self, visitor: &mut V)
        where V: FnMut(&P)
    {
        for attractor in self.attractors.iter() {
            visitor(&attractor.position)
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
                let mut nearest_distance = ap.attract_dist;
                let mut connect_node: Option<&mut Node<_, _>> = None;
                for node in nodes.iter_mut() {
                    if !node.is_active() {
                        // The node has become inactive
                        continue;
                    }

                    let dist = SqDist(node.position.sqdist(&ap.position));

                    if dist < ap.connect_dist {
                        // This node is within the connect radius of a node.
                        // XXX: There might be a closer node, but we use
                        // the first we find.
                        connect_node = Some(node);
                        // outside the node loop, we perform some action
                        break;
                    } else if dist < nearest_distance {
                        // ```node``` is within the influence of the attraction point,
                        // and it's closer than the currently closest node.
                        nearest_distance = dist;
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
                self.add_node(new_position, Some(NodeIdx(i as u32, 0)));

                // and reset growth attraction forces
                self.nodes[i].growth = Zero::zero();
                self.nodes[i].growth_count = 0;
            }
        }

        // Note that nodes can oscillate, between two attraction points, so
        // it's better to stop after a certain number of iterations
        return self.nodes.len() - num_nodes;
    }
}
