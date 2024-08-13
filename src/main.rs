#[warn(unused_imports)]
extern crate nalgebra as na;
use na::base::Matrix4;
use na::{Rotation3, Vector3};

struct SE3 {
    data: Matrix4<f32>,
}

impl SE3 {
    fn new() -> SE3 {
        SE3 {
            data: Matrix4::<f32>::identity(),
        }
    }
    fn rpy(roll: f32, pitch: f32, yaw: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        let rot = Rotation3::from_euler_angles(roll, pitch, yaw);
        transform
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&rot.matrix());
        SE3 { data: transform }
    }
    fn rx(angle: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        let rot = Rotation3::from_euler_angles(angle, 0.0, 0.0);
        transform
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&rot.matrix());
        SE3 { data: transform }
    }
    fn ry(angle: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        let rot = Rotation3::from_euler_angles(0.0, angle, 0.0);
        transform
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&rot.matrix());
        SE3 { data: transform }
    }
    fn rz(angle: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        let rot = Rotation3::from_euler_angles(0.0, 0.0, angle);
        transform
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&rot.matrix());
        SE3 { data: transform }
    }
    fn tx(d: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        transform[(0, 3)] = d;
        SE3 { data: transform }
    }
    fn ty(d: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        transform[(1, 3)] = d;
        SE3 { data: transform }
    }
    fn tz(d: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        transform[(2, 3)] = d;
        SE3 { data: transform }
    }
}

enum ET {
    I,
    Tx {
        d: f32,
    },
    Ty {
        d: f32,
    },
    Tz {
        d: f32,
    },
    Rx {
        angle: f32,
    },
    Ry {
        angle: f32,
    },
    Rz {
        angle: f32,
        isjoint: bool,
        index: i32,
    },
}

struct ETS {
    ets:Vec<ET>
    // add more parameters
    // n -> number of dofs
    // joints -> indices of all joint transformations
    // ...
}

impl ETS {
    fn eval(q: Vec<f32>, base: SE3, tool: SE3) -> SE3 {
        SE3::new()
    }
    fn new(et: ET) -> ETS {
        ETS { ets : vec![et] }
    }
}

fn main() {
    // let t = Matrix4::<f32>::identity();
    let t = SE3::new();
    println!("t = {}", t.data);

    // let axisangle = Vector3::y() * std::f32::consts::FRAC_PI_2;
    // let axisangle = Vector3::y() * 0.0f32;
    // let rot = Rotation3::new(axisangle);
    // println!("t = {}", rot);

    let r = SE3::rx(std::f32::consts::FRAC_PI_2);
    println!("r = {}", r.data);

    // let t = SE3::tx(1.2);
    // println!("t = {}", t.data);
    // let t = SE3::ty(1.2);
    // println!("t = {}", t.data);
    // let t = SE3::tz(1.2);
    // println!("t = {}", t.data);
}
