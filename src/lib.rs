extern crate nalgebra as na;
use na::base::Matrix4;
use na::{Rotation3, Vector3};
use std::ops::Mul;

#[derive(Clone)]
pub struct SE3 {
    data: Matrix4<f32>,
}


// impl std::ops::Mul<SE3> for &SE3 {
//     type Output = SE3;
//     fn mul(self, rhs: SE3) -> SE3 {
//         //SE3::from_matrix(&self.clone().matrix() * rhs.matrix())
//         SE3 { data: self.data * rhs.matrix() }
//     }
// }

// pub trait Mul<Rhs = Self> {
//     type Output;
//     // Required method
//     fn mul(self, rhs: Rhs) -> Self::Output;
// }


impl Mul for SE3 {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        Self {data: self.data * other.data}
    }
}

impl std::fmt::Display for SE3 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self.data)
    }
}

impl SE3 {
    pub fn new() -> SE3 {
        SE3 {
            data: Matrix4::<f32>::identity(),
        }
    }
    pub fn from_matrix(m: Matrix4<f32>) -> SE3 {
        SE3 { data: m.clone() }
    }
    pub fn rpy(roll: f32, pitch: f32, yaw: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        let rot = Rotation3::from_euler_angles(roll, pitch, yaw);
        transform
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&rot.matrix());
        SE3 { data: transform }
    }
    pub fn rx(angle: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        let rot = Rotation3::from_euler_angles(angle, 0.0, 0.0);
        transform
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&rot.matrix());
        SE3 { data: transform }
    }
    pub fn ry(angle: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        let rot = Rotation3::from_euler_angles(0.0, angle, 0.0);
        transform
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&rot.matrix());
        SE3 { data: transform }
    }
    pub fn rz(angle: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        let rot = Rotation3::from_euler_angles(0.0, 0.0, angle);
        transform
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&rot.matrix());
        SE3 { data: transform }
    }
    pub fn tx(d: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        transform[(0, 3)] = d;
        SE3 { data: transform }
    }
    pub fn ty(d: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        transform[(1, 3)] = d;
        SE3 { data: transform }
    }
    pub fn tz(d: f32) -> SE3 {
        let mut transform = Matrix4::<f32>::identity();
        transform[(2, 3)] = d;
        SE3 { data: transform }
    }

    pub fn matrix(&self) -> Matrix4<f32> {
        self.data.clone()
    }
}

#[derive(Copy, Clone)]
pub enum ET {
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
        isjoint: bool, // TODO: use Maybe<usize> for joint or not
        index: i32,
    },
}

impl ET {
    pub fn new() -> ET {
        ET::I
    }
    pub fn tx(x: f32) -> ET {
        ET::Tx { d: x }
    }
    pub fn ty(y: f32) -> ET {
        ET::Ty { d: y }
    }
    pub fn tz(z: f32) -> ET {
        ET::Tz { d: z }
    }
    pub fn rx(roll: f32) -> ET {
        ET::Rx { angle: roll }
    }
    pub fn ry(pitch: f32) -> ET {
        ET::Ry { angle: pitch }
    }
    pub fn rz(yaw: f32) -> ET {
        ET::Rz {
            angle: yaw,
            isjoint: false,
            index: -1,
        }
    }
    pub fn eval(&self) -> SE3 {
        match *self {
            ET::I => SE3::new(),
            ET::Tx { d } => return SE3::tx(d),
            ET::Ty { d } => return SE3::ty(d),
            ET::Tz { d } => return SE3::tz(d),
            ET::Rx { angle } => return SE3::rx(angle),
            ET::Ry { angle } => return SE3::ry(angle),
            // TODO: if angle
            ET::Rz {
                angle,
                isjoint,
                index,
            } => {
                let t = SE3::rz(angle);
                return t;
            }
        }
    }
}

impl std::fmt::Display for ET {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match *self {
            ET::I => write!(f, "SE3()"),
            ET::Tx { d } => write!(f, "Tx({d})"),
            ET::Ty { d } => write!(f, "Ty({d})"),
            ET::Tz { d } => write!(f, "Tz({d})"),
            ET::Rx { angle } => write!(f, "Rx({angle})"),
            ET::Ry { angle } => write!(f, "Ry({angle})"),
            ET::Rz {
                angle,
                isjoint,
                index,
            } => write!(f, "Rz({angle},isjoint={isjoint})"),
        }
    }
}
