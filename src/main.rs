#[warn(unused_imports)]
extern crate nalgebra as na;
use na::base::Matrix4;
use na::{Rotation3, Vector3};

use ets::*;

#[derive(Clone)]
struct ETS {
    // mutable?
    ets: Vec<ET>, // add more parameters
                  // n -> number of dofs
                  // joints -> indices of all joint transformations
                  // ...
}

impl ETS {
    fn eval(&self, q: Vec<f32>, base: SE3, tool: SE3) -> SE3 {
        let mut t = SE3::new();
        for e in self.ets.iter() {
            // chain.ets.push(e.clone())
            t = t * e.eval();
        }
        t
    }
    fn new() -> ETS {
        ETS {
            ets: Vec::<ET>::new(),
        }
    }
    fn from_vec(et: Vec<ET>) -> ETS {
        let mut chain = ETS::new();
        for e in et.iter() {
            chain.ets.push(e.clone())
        }
        chain
    }
}

impl std::fmt::Display for ETS {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        if self.ets.len() == 0 as usize {
            write!(f, "{}", ET::I)?;
            return Ok(());
        }

        for k in 0..(self.ets.len()) {
            let v = self.ets[k];
            write!(f, "{}", v)?;
            if k < (self.ets.len() - 1) {
                // write!(f, " ⋅ ")?;
                write!(f, "⋅")?;
                // write!(f, " ⊗ ")?;
            }
        }
        Ok(())
    }
}

fn main() {
    // let t = Matrix4::<f32>::identity();
    let t = SE3::new();
    println!("t = {}", t);

    // let axisangle = Vector3::y() * std::f32::consts::FRAC_PI_2;
    // let axisangle = Vector3::y() * 0.0f32;
    // let rot = Rotation3::new(axisangle);
    // println!("t = {}", rot);

    let r = SE3::rx(std::f32::consts::FRAC_PI_2);
    println!("r = {}", r);

    // let t = SE3::tx(1.2);
    // println!("t = {}", t);
    // let t = SE3::ty(1.2);
    // println!("t = {}", t);
    // let t = SE3::tz(1.2);
    // println!("t = {}", t);

    let etz = vec![
        ET::I,
        ET::tx(1.0),
        ET::ty(1.0),
        ET::tz(1.0),
        ET::rx(1.0),
        ET::ry(1.0),
        ET::rz(1.0),
    ];

    for et in etz.iter() {
        println!("et = {}", et);
    }

    let etz1 = ETS::new();
    println!("etz1 = {}", etz1);

    let etz1 = ETS::from_vec(etz);
    println!("etz1 = {}", etz1);
}
