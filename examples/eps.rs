extern crate space_colonization;
extern crate nalgebra as na;
extern crate rand;
extern crate num;
extern crate clap;
extern crate eps_writer;

use na::{Pnt2, Vec2, Pnt3, FloatPnt, FloatVec};
use rand::{Rng, Closed01};
use num::Zero;
use clap::{Arg, App};
use std::str::FromStr;
use space_colonization::SpaceColonization;
use eps_writer::{EpsDocument, Position, Points, Lines, SetRGB};
use std::fs::File;

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
    save_every: Option<usize>,
}

impl Config {
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
                          .arg(Arg::with_name("SAVE_EVERY")
                                   .long("save-every")
                                   .help("Save a picture every n iterations (default: none)")
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
            save_every: usize::from_str(matches.value_of("SAVE_EVERY").unwrap_or("INVALID")).ok(),
        }
    }
}

const SCALE: f32 = 400.0;

fn run<T, F>(config: &Config)
    where T: MyPoint + FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    let mut rng = rand::thread_rng();

    let mut sc: SpaceColonization<T, F> = SpaceColonization::new(10, 1);
    for _ in 0..config.n_roots {
        sc.add_root_node(<T as MyPoint>::random(&mut rng));
    }

    for _ in 0..config.n_attraction_points {
        sc.add_attractor(<T as MyPoint>::random(&mut rng));
    }

    let mut i = 0;

    loop {
        if let Some(n) = config.save_every {
            // save current iteration as eps
            if i % n == 0 {
                let filename = format!("out_{:05}.eps", i);
                let mut document = EpsDocument::new();

                let points: Vec<_> = sc.attractors().iter().map(|&pt| {
                     let pnt = pt.into_pnt3();
                     Position::new(pnt.x, pnt.y)
                }).collect();

                document.add_shape(Box::new(Points(points, 0.005*SCALE/2.0)));

                let mut lines = Vec::new();
                sc.iter_segments(&mut |&a, &b| {
                    let pt1 = a.into_pnt3();
                    let pt2 = b.into_pnt3();
                    lines.push((Position::new(pt1.x, pt1.y), Position::new(pt2.x, pt2.y)));
                });
                document.add_shape(Box::new(SetRGB(1.0, 0.0, 0.0)));
                document.add_shape(Box::new(Lines(lines)));

                document.transform(Vec2::new(1.0, 1.0), Vec2::new(SCALE/2.0, SCALE/2.0));

                let mut file = File::create(filename).unwrap();
                document.write_eps(&mut file, 100.0, 100.0).unwrap();
            }
        }

        if let Some(m) = config.max_iter {
            if i > m {
                break;
            }
        }

        let new_nodes = sc.iterate(config.influence_radius,
                                   config.move_distance,
                                   config.kill_distance,
                                   None);

        println!("Iteration: {}. New nodes: {}", i, new_nodes);

        i += 1;
    }
}

fn main() {
    let config = Config::from_cmd();

    println!("{:?}", config);
    assert!(config.use_3d == false);
    run::<Pnt2<f32>, Vec2<f32>>(&config);
}
