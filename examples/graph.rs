// Constructs a graph from a space-colonization simulation

extern crate space_colonization;
extern crate nalgebra as na;
extern crate kiss3d;
extern crate rand;
extern crate num;
extern crate clap;
extern crate dot;

use kiss3d::window::Window;
use na::{Pnt2, Pnt3, Vec2, Vec3, FloatPnt, FloatVec};
use num::Zero;
use space_colonization::{SpaceColonization, SqDist, Attractor, ConnectAction};
use common::{MyPoint, Config};
use std::fmt::Debug;

pub mod common;

#[derive(Debug, Copy, Clone)]
enum Information {
    None,
    Source(usize),
    Target(usize)
}

impl Default for Information {
    fn default() -> Self {
        Information::None
    }
}

fn run<T, F>(config: &Config)
    where T: MyPoint + FloatPnt<f32, F> + Debug,
          F: FloatVec<f32> + Zero + Copy + Debug
{
    let mut rng = rand::thread_rng();

    let mut sc: SpaceColonization<T, F, Information> =
        SpaceColonization::new(SqDist::from_dist(config.influence_radius),
        SqDist::from_dist(config.kill_distance),
        config.max_length,
        config.max_branches,
        config.move_distance);

    // these are the source nodes. This is where we start growing.
    /*
    for src in 0..config.n_roots {
        //sc.add_root_node_with_information(<T as MyPoint>::random(&mut rng), Some(Information::Source(src)));
        sc.add_root_node(<T as MyPoint>::random(&mut rng));
    }
    */

    // add target nodes and their attractor points
    for dst in 0..config.target_nodes.unwrap() {
        let target_pt = <T as MyPoint>::random(&mut rng);

        // For each target node, add a root node.
        // XXX: Treat target and root nodes equally.
        let root_idx = sc.add_root_node(target_pt); 

        // place n attractor points around the target_pt
        for _ in 0 .. config.attractors_per_target_node {
            sc.add_attractor(Attractor {
                attract_dist: SqDist::from_dist(config.influence_radius),
                connect_dist: SqDist::from_dist(config.kill_distance),
                strength: 1.0,
                position: <T as MyPoint>::random_around(&mut rng, target_pt, config.target_attractor_radius),
                information: Information::Target(dst),
                //connect_action: ConnectAction::KillAttractor,
                connect_action: ConnectAction::DisableForConnectingRoot, //{iterations:100_000},
                active_from_iteration: 0,
                not_for_root: Some(root_idx),
                not_for_connecting_root: None,
            });
        }
        // target nodes do not exist. but their attractor points. later we want to generate only
        // nodes, which can be both source and target. we have to take care that a source node
        // is not attracted by itself. that could be accomplished by disabling the attraction
        // points for some time and activating only the nodes created during the last e.g. 5 iterations.

    }

    // these are ordinary attraction points
    for _ in 0..config.n_attraction_points {
        sc.add_default_attractor(<T as MyPoint>::random(&mut rng));
    }

    let mut window = Window::new("Space Colonization Graph");

    let norm_col = Pnt3::new(0.6, 0.6, 0.6);
    let src_col = Pnt3::new(0.0, 1.0, 0.0);
    let dst_col = Pnt3::new(1.0, 0.0, 0.0);

    let line_col = Pnt3::new(0.2, 0.2, 0.2);

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

        sc.visit_node_segments(&mut |&a, &b| {
            window.draw_line(&a.into_pnt3(), &b.into_pnt3(), &line_col)
        });

        sc.visit_attractors(&mut |a| {
            let color =
                match a.information {
                    Information::Target(_) => &dst_col,
                    _ => &norm_col,
                };
            window.draw_point(&a.position.into_pnt3(), color)
        });

        /*
        sc.visit_root_nodes(&mut |a| {
            let color = &src_col;
            window.draw_point(&a.position.into_pnt3(), color)
        });
        */

        let new_nodes = sc.next();

        //println!("Iteration: {}. New nodes: {:?}", i, new_nodes);

        i += 1;
    }

    // Now build the graph. For that we inspect all node that carry information.
    let mut edges = Vec::new();
    println!("digraph space {{");
    sc.visit_nodes_with_info_and_root(&mut|info_node, root_node| {
        match info_node.assigned_information {
            Some(Information::Target(tgt)) => {
                let src = root_node.root.0;
                /*
                println!("tgt: {:?}", tgt);
                println!("info: {:?}", info_node);
                println!("root: {:?}", root_node);
                println!("{} -> {} ({})", src, tgt, info_node.length);
                println!("----------------------------");
                */
                edges.push((src, tgt, info_node.length));

                println!("{} -> {} [weight={}];", src, tgt, info_node.length);
            }
            _ => {}
        }
    });
    //println!("{:?}", edges);


    println!("}}");
}


fn main() {
    let config = Config::from_cmd();

    //println!("{:?}", config);

    if config.use_3d {
        run::<Pnt3<f32>, Vec3<f32>>(&config);
    } else {
        run::<Pnt2<f32>, Vec2<f32>>(&config);
    }
}
