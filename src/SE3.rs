use nalgebra::{Matrix3, Matrix4, Matrix6, Rotation3, Translation3, Vector3, Vector6};


use crate::utils;

#[derive(Clone)]
pub struct SE3 {
    data: Matrix4<f64>,
}

impl std::fmt::Display for SE3 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self.matrix())
    }
}

impl std::ops::Mul for SE3 {
    type Output = SE3;

    fn mul(self, rhs: SE3) -> SE3 {
        SE3::from_matrix(self.matrix() * rhs.matrix())
    }
}

impl std::ops::Mul<&SE3> for &SE3 {
    type Output = SE3;

    fn mul(self, rhs: &SE3) -> SE3 {
        SE3::from_matrix(self.matrix() * rhs.matrix())
    }
}

impl SE3 {
    pub fn new() -> SE3 {
        SE3 {
            data: Matrix4::<f64>::identity(),
        }
    }
    pub fn from_matrix(m: Matrix4<f64>) -> SE3 {
        SE3 { data: m }
    }

    pub fn rx(angle: f64) -> SE3 {
        SE3 {
            data: Rotation3::from_axis_angle(&Vector3::x_axis(), angle).to_homogeneous(),
        }
    }
    pub fn ry(angle: f64) -> SE3 {
        SE3 {
            data: Rotation3::from_axis_angle(&Vector3::y_axis(), angle).to_homogeneous(),
        }
    }
    pub fn rz(angle: f64) -> SE3 {
        SE3 {
            data: Rotation3::from_axis_angle(&Vector3::z_axis(), angle).to_homogeneous(),
        }
    }
    pub fn tx(d: f64) -> SE3 {
        SE3 {
            data: Translation3::new(d, 0.0, 0.0).to_homogeneous(),
        }
    }
    pub fn ty(d: f64) -> SE3 {
        SE3 {
            data: Translation3::new(0.0, d, 0.0).to_homogeneous(),
        }
    }
    pub fn tz(d: f64) -> SE3 {
        SE3 {
            data: Translation3::new(0.0, 0.0, d).to_homogeneous(),
        }
    }
    pub fn transl(x: f64, y: f64, z: f64) -> SE3 {
        SE3 {
            data: Translation3::new(x, y, z).to_homogeneous(),
        }
    }
    pub fn rpy(roll: f64, pitch: f64, yaw: f64) -> SE3 {
        SE3 {
            data: Rotation3::from_euler_angles(roll, pitch, yaw).to_homogeneous(),
        }
    }

    pub fn xyzrpy(x: f64, y: f64, z: f64, roll: f64, pitch: f64, yaw: f64) -> SE3 {
        SE3 {
            data: Translation3::new(x, y, z).to_homogeneous()
                * Rotation3::from_euler_angles(roll, pitch, yaw).to_homogeneous(),
        }
    }

    pub fn matrix(&self) -> Matrix4<f64> {
        self.data.clone()
    }

    pub fn skew(v: Vector6<f64>) -> Matrix4<f64> {
        return utils::skew_twist(v);
    }

    pub fn adjoint(&self) -> Matrix6<f64> {
        let rotation = self.data.fixed_view::<3, 3>(0, 0);
        let translation = self.data.fixed_view::<3, 1>(0, 3);

        // Create the skew-symmetric matrix of the translation vector
        let skew_translation = Matrix3::new(
            0.0,
            -translation.z,
            translation.y,
            translation.z,
            0.0,
            -translation.x,
            -translation.y,
            translation.x,
            0.0,
        );

        // Adjoint matrix is a 6x6 matrix
        let mut adj = Matrix6::zeros();

        // Set the top-left 3x3 as the rotation matrix
        adj.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);

        // Set the top-right 3x3 as skew(translation) * rotation
        adj.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(skew_translation * rotation));

        // Set the bottom-right 3x3 as the rotation matrix
        adj.fixed_view_mut::<3, 3>(3, 3).copy_from(&rotation);
        adj
    }

    pub fn rotation_matrix(&self) -> Matrix3<f64> {
        self.data.fixed_view::<3, 3>(0, 0).into()
    }

    pub fn translation(&self) -> Vector3<f64> {
        self.data.fixed_view::<3, 1>(0, 3).into()
    }

    pub fn inv(&self) -> SE3 {
        let rotation = self.data.fixed_view::<3, 3>(0, 0);
        let rotation_inv = rotation.transpose();

        let translation = self.data.fixed_view::<3, 1>(0, 3);
        let translation_inv = -rotation_inv * translation;

        let mut inverse_matrix = Matrix4::identity();

        inverse_matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&rotation_inv);
        inverse_matrix
            .fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&translation_inv);
        SE3 {
            data: inverse_matrix,
        }
    }
}

pub fn angle_axis(t: &SE3, tdesired: &SE3) -> Vector6<f64> {
    let mut residual = Vector6::zeros();
    // translation error
    let translation_residual = tdesired.translation() - t.translation();
    // rotation error
    let rotation_residual_vec;
    let rotation_residual_matrix = tdesired.rotation_matrix() * t.rotation_matrix().transpose();
    let var = Vector3::from_vec(vec![
        rotation_residual_matrix[(2, 1)] - rotation_residual_matrix[(1, 2)],
        rotation_residual_matrix[(0, 2)] - rotation_residual_matrix[(2, 0)],
        rotation_residual_matrix[(1, 0)] - rotation_residual_matrix[(0, 1)],
    ]);
    let rotation_residual_matrix_trace = rotation_residual_matrix.trace();
    let var_norm = var.norm();

    if var_norm < 1e-6 {
        if rotation_residual_matrix_trace > 0f64 {
            rotation_residual_vec = Vector3::zeros();
        } else {
            rotation_residual_vec = Vector3::from_vec(vec![
                std::f64::consts::FRAC_PI_2 * (rotation_residual_matrix[(0, 0)] + 1f64),
                std::f64::consts::FRAC_PI_2 * (rotation_residual_matrix[(1, 1)] + 1f64),
                std::f64::consts::FRAC_PI_2 * (rotation_residual_matrix[(2, 2)] + 1f64),
            ]);
        }
    } else {
        let angle = var_norm.atan2(rotation_residual_matrix_trace - 1f64);
        rotation_residual_vec = (angle / var_norm) * var;
    }
    residual
        .fixed_rows_mut::<3>(3)
        .copy_from(&rotation_residual_vec);
    residual
        .fixed_rows_mut::<3>(0)
        .copy_from(&translation_residual);
    residual
}