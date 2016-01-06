extern crate nalgebra as na;
extern crate kiss3d;
extern crate rand;
extern crate num;

use kiss3d::window::Window;
use na::{Pnt2, Pnt3, Norm, FloatPnt, FloatVec};
use rand::{Rng, Closed01};
use num::Zero;

struct SpaceColonization<T, F>
    where T: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    node_parents: Vec<usize>,
    node_positions: Vec<T>,
    leaf_node_pointers: Vec<usize>, // XXX: how can a branch be terminated?
    leaf_node_grow_directions: Vec<F>,
}

impl<T, F> SpaceColonization<T, F>
    where T: FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    fn new() -> SpaceColonization<T, F> {
        SpaceColonization {
            node_parents: Vec::new(),
            node_positions: Vec::new(),
            leaf_node_pointers: Vec::new(),
            leaf_node_grow_directions: Vec::new(),
        }
    }

    // adds initial root node(s)
    fn add_root_node(&mut self, pos: T) {
        debug_assert!(self.node_positions.len() == self.node_parents.len()); // invariant
        debug_assert!(self.leaf_node_pointers.len() == self.leaf_node_grow_directions.len()); // invariant

        let idx = self.node_positions.len();
        self.node_positions.push(pos);

        // a root node has it's own index as parent
        self.node_parents.push(idx);

        // a root node creates a new "leaf" node (better: branch!)
        self.leaf_node_pointers.push(idx);
        self.leaf_node_grow_directions.push(Zero::zero());

        debug_assert!(self.node_positions.len() == self.node_parents.len()); // invariant
        debug_assert!(self.leaf_node_pointers.len() == self.leaf_node_grow_directions.len()); // invariant
    }

    fn add_moved_leaf_node(&mut self, leaf_node_slot: usize, v: F) {
        let parent = self.leaf_node_pointers[leaf_node_slot];
        let parent_pos = self.node_positions[parent];

        self.update_leaf_node(leaf_node_slot, parent_pos + v)
    }

    fn update_leaf_node(&mut self, leaf_node_slot: usize, new_pos: T) {
        let parent = self.leaf_node_pointers[leaf_node_slot];

        // create new node
        let idx = self.node_positions.len();
        self.node_positions.push(new_pos);
        self.node_parents.push(parent);

        // update the leaf node pointer
        self.leaf_node_pointers[leaf_node_slot] = idx;
    }

    fn find_nearest_leaf_for_attraction_point(&self,
                                              attraction_pt: &T,
                                              radius_influence_sq: f32)
                                              -> Option<(usize, f32)> {
        let mut nearest_leaf: Option<usize> = None;
        let mut nearest_distance_sq: f32 = radius_influence_sq;

        for (i, &leaf_node_ptr) in self.leaf_node_pointers.iter().enumerate() {
            let leaf_pos = &self.node_positions[leaf_node_ptr];
            let dist_sq = leaf_pos.sqdist(attraction_pt);

            // is the attraction point within the radius of influence
            // and closed than the currently best
            if dist_sq < nearest_distance_sq {
                nearest_distance_sq = dist_sq;
                nearest_leaf = Some(i);
            }
        }

        return nearest_leaf.map(|idx| (idx, nearest_distance_sq));
    }

    fn iter_segments<C>(&self, callback: &mut C)
        where C: FnMut(&T, &T)
    {
        for (i, &parent) in self.node_parents.iter().enumerate() {
            if i != parent {
                callback(&self.node_positions[i], &self.node_positions[parent]);
            }
        }
    }

    fn iterate(&mut self,
               attraction_points: &mut [(T, bool)],
               radius_influce_sq: f32,
               move_distance: f32,
               kill_distance_sq: f32) {
        // reset all attraction forces
        for force in &mut self.leaf_node_grow_directions {
            *force = Zero::zero();
        }

        // for each attraction_point, find the nearest (leaf) node that it
        // influences.
        for ap in attraction_points.iter_mut() {
            let active = ap.1;
            if !active {
                continue;
            }
            let nearest_leaf_opt = self.find_nearest_leaf_for_attraction_point(&ap.0,
                                                                               radius_influce_sq);
            if let Some((nearest_leaf_slot, d_sq)) = nearest_leaf_opt {
                let nearest_leaf_node_idx = self.leaf_node_pointers[nearest_leaf_slot];
                // update the force with the normalized vector towards the attraction point
                let v = (ap.0 - self.node_positions[nearest_leaf_node_idx]).normalize();
                self.leaf_node_grow_directions[nearest_leaf_slot] =
                    self.leaf_node_grow_directions[nearest_leaf_slot] + v;

                if d_sq < kill_distance_sq {
                    // set inactive
                    ap.1 = false;
                }
            }
        }

        // now create new leaf nodes
        for i in 0..self.leaf_node_grow_directions.len() {
            let force = self.leaf_node_grow_directions[i];

            // update leaf node only if there is a force!
            if !force.is_zero() {
                let n = force.normalize();
                self.add_moved_leaf_node(i, n * move_distance);
            }
        }

    }
}

fn random_coord<R: Rng>(rng: &mut R) -> f32 {
    2.0 * rng.gen::<Closed01<f32>>().0 - 1.0
}

fn random_point<R: Rng>(rng: &mut R) -> Pnt2<f32> {
    Pnt2::new(random_coord(rng), random_coord(rng))
}

const Z: f32 = 3.0;

fn main() {
    let mut rng = rand::thread_rng();

    let n_attraction_points = 5000;

    // generate initial attraction point configuration
    // stores status of attraction point in the second tuple element (true=active, false=used)
    let mut attraction_points: Vec<_> = (0..n_attraction_points)
                                            .into_iter()
                                            .map(|_| (random_point(&mut rng), true))
                                            .collect();

    let mut sc = SpaceColonization::new();
    sc.add_root_node(random_point(&mut rng));
    sc.add_root_node(random_point(&mut rng));
    sc.add_root_node(random_point(&mut rng));

    let mut window = Window::new("Space Colonization");
    let white = Pnt3::new(1.0, 1.0, 1.0);

    let mut i = 0;

    while window.render() {
        i += 1;
        println!("Iteration: {}", i);
        for &(ref pt, status) in &attraction_points {
            if status {
                window.draw_point(&Pnt3::new(pt.x, pt.y, Z), &white);
            }
        }

        sc.iter_segments(&mut |a, b| {
            window.draw_line(&Pnt3::new(a.x, a.y, Z), &Pnt3::new(b.x, b.y, Z), &white);
        });

        sc.iterate(&mut attraction_points, 0.1 * 0.1, 0.01, 0.005);
    }
}
