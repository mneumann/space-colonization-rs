use rand::{Rng, Closed01};
use std::str::FromStr;
use clap::{Arg, App};
use na::{Pnt2, Pnt3};

fn random_closed01<R: Rng>(rng: &mut R) -> f32 {
    rng.gen::<Closed01<f32>>().0
}

fn random_coord<R: Rng>(rng: &mut R) -> f32 {
    2.0 * random_closed01(rng) - 1.0
}

pub trait MyPoint {
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
pub struct Config {
    pub n_attraction_points: usize,
    pub n_roots: usize,
    pub influence_radius: f32,
    pub move_distance: f32,
    pub kill_distance: f32,
    pub use_3d: bool,
    pub max_iter: Option<usize>,
    pub save_every: Option<usize>,
    pub max_length: u32,
    pub max_branches: u32,
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
            save_every: None,
            max_length: 100,
            max_branches: 10,
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
            save_every: None,
            max_length: 100,
            max_branches: 10,
        }
    }

    pub fn from_cmd() -> Config {
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
                                   .help("Move distance (default: 0.05)")
                                   .takes_value(true)
                                   .required(false))
                          .arg(Arg::with_name("MAX_LENGTH")
                                   .long("max-length")
                                   .help("Maximal allowed length from root to leaf (default: 100)")
                                   .takes_value(true)
                                   .required(false))
                          .arg(Arg::with_name("MAX_BRANCHES")
                                   .long("max-branches")
                                   .help("Maximal allowed number of branches per node (default: 10)")
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
            n_attraction_points: FromStr::from_str(matches.value_of("NUM_POINTS").unwrap_or("1000"))
                                     .unwrap(),
            n_roots: FromStr::from_str(matches.value_of("NUM_ROOTS").unwrap_or("1")).unwrap(),
            influence_radius: FromStr::from_str(matches.value_of("IR").unwrap_or("0.25")).unwrap(),
            kill_distance: FromStr::from_str(matches.value_of("KD").unwrap_or("0.1")).unwrap(),
            move_distance: FromStr::from_str(matches.value_of("MD").unwrap_or("0.05")).unwrap(),
            use_3d: matches.is_present("USE_3D"),
            max_iter: FromStr::from_str(matches.value_of("MAX_ITER").unwrap_or("INVALID")).ok(),
            max_length: FromStr::from_str(matches.value_of("MAX_LENGTH").unwrap_or("100")).unwrap(),
            max_branches: FromStr::from_str(matches.value_of("MAX_BRANCHES").unwrap_or("10")).unwrap(),
            save_every: FromStr::from_str(matches.value_of("SAVE_EVERY").unwrap_or("INVALID")).ok(),
        }
    }
}
