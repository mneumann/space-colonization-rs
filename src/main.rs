extern crate nalgebra as na;
extern crate kiss3d;
extern crate rand;
extern crate num;
extern crate clap;

use kiss3d::window::Window;
use na::{Pnt2, Pnt3, Vec2, Vec3, Norm, FloatPnt, FloatVec};
use rand::{Rng, Closed01};
use num::Zero;
use clap::{Arg, App};
use std::str::FromStr;

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
    fn new() -> SpaceColonization<T, F> {
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

    fn iter_segments<C>(&self, callback: &mut C)
        where C: FnMut(&T, &T)
    {
        for (i, node) in self.nodes.iter().enumerate() {
            if i != node.parent {
                callback(&node.position, &self.nodes[node.parent].position);
            }
        }
    }

    fn iterate(&mut self,
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

fn random_closed01<R: Rng>(rng: &mut R) -> f32 {
    rng.gen::<Closed01<f32>>().0
}

fn random_coord<R: Rng>(rng: &mut R) -> f32 {
    2.0 * random_closed01(rng) - 1.0
}

trait MyPoint {
    fn into_pnt3(self) -> Pnt3<f32>;
    fn random<R: Rng>(rng: &mut R) -> Self;
}

impl MyPoint for Pnt3<f32> {
    fn into_pnt3(self) -> Pnt3<f32> {
        self
    }

    fn random<R: Rng>(rng: &mut R) -> Pnt3<f32> {
        Pnt3::new(random_coord(rng), random_coord(rng), random_coord(rng))
    }
}

impl MyPoint for Pnt2<f32> {
    fn into_pnt3(self) -> Pnt3<f32> {
        Pnt3::new(self.x, self.y, 3.0)
    }

    fn random<R: Rng>(rng: &mut R) -> Pnt2<f32> {
        Pnt2::new(random_coord(rng), random_coord(rng))
    }
}

#[derive(Debug)]
struct Config {
    n_attraction_points: usize,
    n_roots: usize,
    influence_radius: f32,
    move_distance: f32,
    kill_distance: f32,
    use_3d: bool,
    max_iter: Option<usize>,
}

impl Config {
    #[allow(dead_code)]
    fn config1() -> Config {
        Config {
            n_attraction_points: 10_000,
            n_roots: 5,
            influence_radius: 0.44,
            kill_distance: 0.22,
            move_distance: 0.02,
            use_3d: true,
            max_iter: None,
        }
    }

    #[allow(dead_code)]
    fn config2() -> Config {
        Config {
            n_attraction_points: 10_000,
            n_roots: 5,
            influence_radius: 0.1,
            kill_distance: 0.07,
            move_distance: 0.01,
            use_3d: false,
            max_iter: None,
        }
    }

    fn from_cmd() -> Config {
        let matches = App::new("space-colonization")
                          .arg(Arg::with_name("NUM_POINTS")
                                   .long("num-points")
                                   .help("Number of attraction points (default: 1000)")
                                   .takes_value(true)
                                   .required(false))
                          .arg(Arg::with_name("NUM_ROOTS")
                                   .long("num-roots")
                                   .help("Number of root nodes (default: 1)")
                                   .takes_value(true)
                                   .required(false))
                          .arg(Arg::with_name("IR")
                                   .long("radius")
                                   .help("Influence radius (default: 0.25)")
                                   .takes_value(true)
                                   .required(false))
                          .arg(Arg::with_name("KD")
                                   .long("kill-distance")
                                   .help("Kill distance (default: 0.1)")
                                   .takes_value(true)
                                   .required(false))
                          .arg(Arg::with_name("MD")
                                   .long("move-distance")
                                   .help("Kill distance (default: 0.05)")
                                   .takes_value(true)
                                   .required(false))
                          .arg(Arg::with_name("MAX_ITER")
                                   .long("max-iter")
                                   .help("Maximum iterations (default: infinite)")
                                   .takes_value(true)
                                   .required(false))
                          .arg(Arg::with_name("USE_3D")
                                   .long("use-3d")
                                   .help("Use 3d mode"))
                          .get_matches();

        Config {
            n_attraction_points: usize::from_str(matches.value_of("NUM_POINTS").unwrap_or("1000"))
                                     .unwrap(),
            n_roots: usize::from_str(matches.value_of("NUM_ROOTS").unwrap_or("1")).unwrap(),
            influence_radius: f32::from_str(matches.value_of("IR").unwrap_or("0.25")).unwrap(),
            kill_distance: f32::from_str(matches.value_of("KD").unwrap_or("0.1")).unwrap(),
            move_distance: f32::from_str(matches.value_of("MD").unwrap_or("0.05")).unwrap(),
            use_3d: matches.is_present("USE_3D"),
            max_iter: usize::from_str(matches.value_of("MAX_ITER").unwrap_or("INVALID")).ok(),
        }
    }
}


fn run<T, F>(config: &Config)
    where T: MyPoint + FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    let mut rng = rand::thread_rng();

    // generate initial attraction point configuration
    // stores status of attraction point in the second tuple element (true=active, false=used)
    let mut attraction_points: Vec<(T, bool)> = (0..config.n_attraction_points)
                                                    .into_iter()
                                                    .map(|_| {
                                                        (<T as MyPoint>::random(&mut rng), true)
                                                    })
                                                    .collect();

    let mut sc: SpaceColonization<T, F> = SpaceColonization::new();
    for _ in 0..config.n_roots {
        sc.add_root_node(<T as MyPoint>::random(&mut rng));
    }

    let mut window = Window::new("Space Colonization");
    let white = Pnt3::new(1.0, 1.0, 1.0);
    let red = Pnt3::new(1.0, 0.0, 0.0);

    let mut i = 0;

    while window.render() {
        i += 1;

        if let Some(m) = config.max_iter {
            if i > m {
                break;
            }
        }

        println!("Iteration: {}", i);
        for &(pt, status) in &attraction_points {
            if status {
                window.draw_point(&pt.into_pnt3(), &white);
            }
        }

        sc.iter_segments(&mut |&a, &b| {
            window.draw_line(&a.into_pnt3(), &b.into_pnt3(), &red);
        });

        let new_nodes = sc.iterate(&mut attraction_points,
                                   config.influence_radius * config.influence_radius,
                                   config.move_distance,
                                   config.kill_distance * config.kill_distance);

        if new_nodes == 0 {
            break;
        }
    }
}


fn main() {
    let config = Config::from_cmd();

    println!("{:?}", config);

    if config.use_3d {
        run::<Pnt3<f32>, Vec3<f32>>(&config);
    } else {
        run::<Pnt2<f32>, Vec2<f32>>(&config);
    }
}
