extern crate space_colonization;
extern crate nalgebra as na;
extern crate rand;
extern crate num;
extern crate clap;
extern crate eps_writer;

use na::{Pnt2, Vec2, FloatPnt, FloatVec};
use num::Zero;
use space_colonization::{SpaceColonization, SqDist};
use eps_writer::{EpsDocument, Position, Points, Lines, SetRGB};
use std::fs::File;
use common::{MyPoint, Config};
use std::fmt::Debug;

pub mod common;

const SCALE: f32 = 400.0;

fn run<T, F>(config: &Config)
    where T: MyPoint + FloatPnt<f32, F> + Debug,
          F: FloatVec<f32> + Zero + Copy + Debug
{
    let mut rng = rand::thread_rng();

    let mut sc: SpaceColonization<T, F, ()> =
        SpaceColonization::new(SqDist::from_dist(config.influence_radius),
                               SqDist::from_dist(config.kill_distance),
                               100, // max_length
                               2, // max_branches
                               config.move_distance);

    for _ in 0..config.n_roots {
        sc.add_root_node(<T as MyPoint>::random(&mut rng));
    }

    for _ in 0..config.n_attraction_points {
        sc.add_default_attractor(<T as MyPoint>::random(&mut rng));
    }

    let mut i = 0;

    loop {
        if let Some(n) = config.save_every {
            // save current iteration as eps
            if i % n == 0 {
                let filename = format!("out_{:05}.eps", i);
                let mut document = EpsDocument::new();

                let mut points = Vec::new();
                sc.visit_attractor_points(&mut|position| {
                                           let pnt = position.into_pnt3();
                                           points.push(Position::new(pnt.x, pnt.y));
                                       });

                document.add_shape(Box::new(Points(points, 0.005 * SCALE / 2.0)));

                let mut lines = Vec::new();
                sc.visit_node_segments(&mut |&a, &b| {
                    let pt1 = a.into_pnt3();
                    let pt2 = b.into_pnt3();
                    lines.push((Position::new(pt1.x, pt1.y), Position::new(pt2.x, pt2.y)));
                });
                document.add_shape(Box::new(SetRGB(1.0, 0.0, 0.0)));
                document.add_shape(Box::new(Lines(lines)));

                document.transform(Vec2::new(1.0, 1.0), Vec2::new(SCALE / 2.0, SCALE / 2.0));

                let mut file = File::create(filename).unwrap();
                document.write_eps(&mut file, 100.0, 100.0).unwrap();
            }
        }

        if let Some(m) = config.max_iter {
            if i > m {
                break;
            }
        }

        let new_nodes = sc.next();

        println!("Iteration: {}. New nodes: {:?}", i, new_nodes);

        i += 1;
    }
}

fn main() {
    let config = Config::from_cmd();

    println!("{:?}", config);
    assert!(config.use_3d == false);
    run::<Pnt2<f32>, Vec2<f32>>(&config);
}
