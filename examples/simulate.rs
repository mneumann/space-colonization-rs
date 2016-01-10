extern crate space_colonization;
extern crate nalgebra as na;
extern crate kiss3d;
extern crate rand;
extern crate num;
extern crate clap;

use kiss3d::window::Window;
use na::{Pnt2, Pnt3, Vec2, Vec3, FloatPnt, FloatVec};
use num::Zero;
use space_colonization::{SpaceColonization, SqDist};
use common::{MyPoint, Config};

pub mod common; 

fn run<T, F>(config: &Config)
    where T: MyPoint + FloatPnt<f32, F>,
          F: FloatVec<f32> + Zero + Copy
{
    let mut rng = rand::thread_rng();

    let mut sc: SpaceColonization<T, F, ()> =
        SpaceColonization::new(SqDist::from_dist(config.influence_radius),
                               SqDist::from_dist(config.kill_distance),
                               config.max_length,
                               config.max_branches,
                               config.move_distance);

    for _ in 0..config.n_roots {
        sc.add_root_node(<T as MyPoint>::random(&mut rng));
    }

    for _ in 0..config.n_attraction_points {
        sc.add_default_attractor(<T as MyPoint>::random(&mut rng));
    }

    let mut window = Window::new("Space Colonization");
    let white = Pnt3::new(1.0, 1.0, 1.0);
    let red = Pnt3::new(1.0, 0.0, 0.0);

    let mut i = 0;

    while window.render() {
        if let Some(n) = config.save_every {
            // save previous iteration of window.render()
            if i > 0 && (i - 1) % n == 0 {
                let img = window.snap_image();
                let filename = format!("out_{:05}.png", i - 1);
                let _ = img.save(&filename).unwrap();
            }
        }
        if let Some(m) = config.max_iter {
            if i > m {
                break;
            }
        }

        sc.visit_attractor_points(&mut |position| window.draw_point(&position.into_pnt3(), &white));

        sc.visit_node_segments(&mut |&a, &b| {
            window.draw_line(&a.into_pnt3(), &b.into_pnt3(), &red)
        });

        let new_nodes = sc.next();

        println!("Iteration: {}. New nodes: {:?}", i, new_nodes);

        i += 1;
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
