extern crate nalgebra as na;
use na::base::Matrix4;
use na::{
    DMatrix, DVector, Matrix, Matrix3, Matrix3x4, Matrix3xX, Matrix6, Matrix6xX, MatrixXx6,
    Rotation3, Translation3, Vector3, Vector6,
};
use std::f64::consts::FRAC_PI_2;
use std::f64::consts::PI;
use std::ops::Mul;

fn skew(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

fn skew_twist(twist: Vector6<f64>) -> Matrix4<f64> {
    // Split the twist vector into angular (omega) and linear (v) components
    let omega = Vector3::new(twist[3], twist[4], twist[5]); // Angular part
    let v = Vector3::new(twist[0], twist[1], twist[2]); // Linear part

    // Construct the skew-symmetric matrix for the twist
    Matrix4::new(
        0.0, -omega.z, omega.y, v.x, omega.z, 0.0, -omega.x, v.y, -omega.y, omega.x, 0.0, v.z, 0.0,
        0.0, 0.0, 0.0,
    )
}

fn rot3(m: Matrix4<f64>) -> Matrix3<f64> {
    return m.fixed_view::<3, 3>(0, 0).into();
}

fn vex(m: Matrix3<f64>) -> Vector3<f64> {
    // Extract the angular velocity components from the skew-symmetric 3x3 submatrix
    let omega_x = m[(2, 1)]; // element (2,1) is -ω_x, so take it directly
    let omega_y = m[(0, 2)]; // element (0,2) is ω_y
    let omega_z = m[(1, 0)]; // element (1,0) is ω_z
                             // Combine into a twist vector: [v_x, v_y, v_z, omega_x, omega_y, omega_z]
    Vector3::new(omega_x, omega_y, omega_z)
}

fn mat_to_twist(m: Matrix4<f64>) -> Vector6<f64> {
    // Extract the angular velocity components from the skew-symmetric 3x3 submatrix
    let omega_x = m[(2, 1)]; // element (2,1) is -ω_x, so take it directly
    let omega_y = m[(0, 2)]; // element (0,2) is ω_y
    let omega_z = m[(1, 0)]; // element (1,0) is ω_z

    // Extract the linear velocity components from the 3x1 translation vector
    let v_x = m[(0, 3)];
    let v_y = m[(1, 3)];
    let v_z = m[(2, 3)];

    // Combine into a twist vector: [v_x, v_y, v_z, omega_x, omega_y, omega_z]
    Vector6::new(v_x, v_y, v_z, omega_x, omega_y, omega_z)
}

/// Represents a 6D Twist (linear and angular velocity).
pub struct Twist {
    pub v: Vector3<f64>,     // Linear velocity
    pub omega: Vector3<f64>, // Angular velocity
}

impl Twist {
    pub fn new(v: Vector3<f64>, omega: Vector3<f64>) -> Self {
        Twist { v, omega }
    }

    /// Combine the linear and angular components into a 6D vector
    pub fn as_vector6(&self) -> Vector6<f64> {
        Vector6::new(
            self.v.x,
            self.v.y,
            self.v.z,
            self.omega.x,
            self.omega.y,
            self.omega.z,
        )
    }
}

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
        return skew_twist(v);
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

// impl std::ops::Mul<SE3> for &SE3 {
//     type Output = SE3;
//     fn mul(self, rhs: SE3) -> SE3 {
//         // SE3 { data: self.matrix() * rhs.matrix(), }
//         SE3::from_matrix(&self.clone().matrix() * rhs.matrix())
//     }
// }

#[derive(Copy, Clone)]
pub enum ET {
    I,
    Tx {
        d: f64,
    },
    Ty {
        d: f64,
    },
    Tz {
        d: f64,
    },
    Rx {
        angle: f64,
    },
    Ry {
        angle: f64,
    },
    Rz {
        angle: f64,
        isjoint: bool, // TODO: use Maybe<usize> for joint or not
        index: i32,
    },
    Transl {
        x: f64,
        y: f64,
        z: f64,
    },
    RPY {
        roll: f64,
        pitch: f64,
        yaw: f64,
    },
    T {
        x: f64,
        y: f64,
        z: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
    },
}

impl ET {
    pub fn new() -> ET {
        ET::I
    }
    pub fn tx(x: f64) -> ET {
        ET::Tx { d: x }
    }
    pub fn ty(y: f64) -> ET {
        ET::Ty { d: y }
    }
    pub fn tz(z: f64) -> ET {
        ET::Tz { d: z }
    }
    pub fn rx(roll: f64) -> ET {
        ET::Rx { angle: roll }
    }
    pub fn ry(pitch: f64) -> ET {
        ET::Ry { angle: pitch }
    }
    pub fn rz(yaw: f64) -> ET {
        ET::Rz {
            angle: yaw,
            isjoint: false,
            index: -1,
        }
    }

    pub fn der(&self) -> Matrix4<f64> {
        let mut m = Matrix4::<f64>::zeros();
        match *self {
            ET::Rx { .. } => {
                m[(1, 2)] = -1f64;
                m[(2, 1)] = 1f64;
            }
            ET::Ry { .. } => {
                m[(0, 2)] = 1f64;
                m[(2, 0)] = -1f64;
            }
            ET::Rz { .. } => {
                m[(0, 1)] = -1f64;
                m[(1, 0)] = 1f64;
            }
            ET::Tx { .. } => {
                m[(0, 3)] = 1f64;
            }
            ET::Ty { .. } => {
                m[(1, 3)] = 1f64;
            }
            ET::Tz { .. } => {
                m[(2, 3)] = 1f64;
            }
            ET::I => {}
            ET::Transl { .. } => {
                m[(0, 3)] = 1f64;
                m[(1, 3)] = 1f64;
                m[(2, 3)] = 1f64;
            }
            ET::RPY { .. } => {
                m[(1, 2)] = -1f64;
                m[(2, 1)] = 1f64;
                m[(0, 2)] = 1f64;
                m[(2, 0)] = -1f64;
                m[(0, 1)] = -1f64;
                m[(1, 0)] = 1f64;
            }
            ET::T { .. } => {
                m[(1, 2)] = -1f64;
                m[(2, 1)] = 1f64;
                m[(0, 2)] = 1f64;
                m[(2, 0)] = -1f64;
                m[(0, 1)] = -1f64;
                m[(1, 0)] = 1f64;
                // translation
                m[(0, 3)] = 1f64;
                m[(1, 3)] = 1f64;
                m[(2, 3)] = 1f64;
            } // _ => return Matrix4::zeros(),
        }
        m
    }

    pub fn eval(&self, eta: f64) -> SE3 {
        match *self {
            ET::I => SE3::new(),
            ET::Tx { d } => return SE3::tx(d),
            ET::Ty { d } => return SE3::ty(d),
            ET::Tz { d } => return SE3::tz(d),
            ET::Rx { angle } => return SE3::rx(angle),
            ET::Ry { angle } => return SE3::ry(angle),
            // TODO: if angle
            ET::Rz { .. } => {
                return SE3::rz(eta);
            }
            ET::Transl { x, y, z } => return SE3::transl(x, y, z),
            ET::RPY { roll, pitch, yaw } => return SE3::rpy(roll, pitch, yaw),
            ET::T {
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
            } => return SE3::xyzrpy(x, y, z, roll, pitch, yaw),
        }
    }
}

impl std::fmt::Display for ET {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match *self {
            ET::I => write!(f, "SE3()"),
            ET::Tx { d } => write!(f, "SE3({d},0,0)"),
            ET::Ty { d } => write!(f, "SE3(0,{d},0)"),
            ET::Tz { d } => write!(f, "SE3(0,0,{d})"),
            ET::Rx { angle } => write!(f, "Rx({angle}°)"),
            ET::Ry { angle } => write!(f, "Ry({angle}°)"),
            ET::Rz {
                angle,
                isjoint,
                index,
            } => write!(f, "Rz({angle}°,isjoint={isjoint},index={index})"),
            // ET::Transl { .. } | ET::RPY { .. } | ET::T { .. } => todo!()
            ET::Transl { x, y, z } => write!(f, "SE3({x},{y},{z})"),
            ET::RPY { roll, pitch, yaw } => {
                let roll_deg = roll.to_degrees();
                let pitch_deg = pitch.to_degrees();
                let yaw_deg = yaw.to_degrees();
                write!(f, "SE3.R({roll_deg:.1}°,{pitch_deg:.1}°,{yaw_deg:.1}°)")
            }
            ET::T {
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
            } => {
                let roll_deg = roll * (180.0 / PI);
                let pitch_deg = pitch * (180.0 / PI);
                let yaw_deg = yaw * (180.0 / PI);
                write!(f, "SE3({x},{y},{z};{roll_deg}°,{pitch_deg}°,{yaw_deg}°)")
            }
        }
    }
}

pub struct ETS {
    pub sequence: Vec<ET>,
}

impl ETS {
    pub fn new(sequence: Vec<ET>) -> Self {
        ETS { sequence }
    }

    pub fn joints_ets_indices(&self) -> Vec<usize> {
        let indices: Vec<_> = self
            .sequence
            .iter()
            .enumerate() // Get index and element
            .filter_map(|(index, et)| {
                if let ET::Rz { isjoint: true, .. } = et {
                    Some(index) // If it's a matching ET::Rz, return the index
                } else {
                    None // If not, skip it
                }
            })
            .collect();
        return indices;
    }

    pub fn ndof(&self) -> usize {
        return self.joints_ets_indices().len();
    }

    pub fn fk(&self, q: &DVector<f64>, base: &Option<SE3>, tool: &Option<SE3>) -> SE3 {
        // Initialize with base transformation if provided, otherwise identity
        let mut result = base.clone().unwrap_or(SE3::new());

        // Apply each transformation in the sequence
        for et in &self.sequence {
            let m = match &et {
                ET::Rz {
                    angle,
                    isjoint,
                    index,
                } => {
                    let mut joint_angle = *angle;
                    if *isjoint {
                        joint_angle += q[*index as usize];
                    }
                    SE3::rz(joint_angle)
                }
                ET::I => SE3::new(),
                ET::Tx { d } => SE3::tx(*d),
                ET::Ty { d } => SE3::ty(*d),
                ET::Tz { d } => SE3::tz(*d),
                ET::Rx { angle } => SE3::rx(*angle),
                ET::Ry { angle } => SE3::ry(*angle),
                ET::Transl { x, y, z } => SE3::transl(*x, *y, *z),
                ET::RPY { roll, pitch, yaw } => SE3::rpy(*roll, *pitch, *yaw),
                ET::T {
                    x,
                    y,
                    z,
                    roll,
                    pitch,
                    yaw,
                } => SE3::xyzrpy(*x, *y, *z, *roll, *pitch, *yaw),
            };
            result = result * m; // Evaluate ET and multiply
        }

        // Apply tool transformation if provided
        if let Some(tool_transform) = tool {
            result = result * tool_transform.clone();
        }

        result
    }

    pub fn joint_jacobian(&self, joint_index: usize, q: &DVector<f64>) -> Matrix4<f64> {
        let dt = Matrix4::<f64>::identity();
        dt
    }

    // pub fn jacobian_rotational_component(&self, joint_index: usize, q: &DVector<f64>, base: Option<SE3>, tool: Option<SE3>) -> Vector3<f64> {
    //     // let mut jacobian = Matrix3xX::zeros(self.ndof());
    //     let end_effector = self.fk(q, base, tool);
    //     vex(rot3(self.joint_jacobian(joint_index, q)) * rot3(end_effector.matrix()).transpose())
    // }

    pub fn jacobian0(&self, q: &DVector<f64>) -> Matrix6xX<f64> {
        let mut jacobian_matrix = Matrix6xX::<f64>::zeros(self.ndof());
        let end_effector = self.fk(q, &None, &None);
        let mut current_frame = SE3::from_matrix(Matrix4::identity());
        for (k, et) in self.sequence.iter().enumerate() {
            match et {
                ET::I => {}
                ET::Transl { x, y, z } => current_frame = current_frame * SE3::transl(*x, *y, *z),
                ET::Tx { d } => current_frame = current_frame * SE3::tx(*d),
                ET::Ty { d } => current_frame = current_frame * SE3::ty(*d),
                ET::Tz { d } => current_frame = current_frame * SE3::tz(*d),
                ET::Rx { angle } => current_frame = current_frame * SE3::rx(*angle),
                ET::Ry { angle } => current_frame = current_frame * SE3::ry(*angle),
                ET::Rz {
                    angle,
                    isjoint,
                    index,
                } => {
                    let mut joint_angle = *angle;
                    if *isjoint {
                        joint_angle = joint_angle + q[*index as usize];
                    }
                    current_frame = current_frame * SE3::rz(joint_angle);
                    let distal_transform = current_frame.inv() * end_effector.clone();
                    let x = distal_transform.data[(0, 3)];
                    let y = distal_transform.data[(1, 3)];
                    // let z = distal_transform.data[(2,3)];
                    let n = current_frame.data.fixed_view::<3, 1>(0, 0);
                    let o = current_frame.data.fixed_view::<3, 1>(0, 1);
                    let a = current_frame.data.fixed_view::<3, 1>(0, 2).into_owned();
                    let joint_rot_jacobian = o * x - n * y;
                    let joint_transl_jacobian = a.clone();
                    // if (*index) % 2 == 0 {
                    //     joint_rot_jacobian = -1f64 * joint_rot_jacobian;
                    //     joint_transl_jacobian = -1f64 * joint_transl_jacobian;
                    // }
                    jacobian_matrix
                        .fixed_view_mut::<3, 1>(0, *index as usize)
                        .copy_from(&joint_rot_jacobian);
                    jacobian_matrix
                        .fixed_view_mut::<3, 1>(3, *index as usize)
                        .copy_from(&joint_transl_jacobian);
                }
                ET::RPY { roll, pitch, yaw } => {
                    current_frame = current_frame * SE3::rpy(*roll, *pitch, *yaw)
                }
                ET::T {
                    x,
                    y,
                    z,
                    roll,
                    pitch,
                    yaw,
                } => current_frame = current_frame * SE3::xyzrpy(*x, *y, *z, *roll, *pitch, *yaw),
            };
        }
        jacobian_matrix
    }

    pub fn jacobian(&self, q: &DVector<f64>, tool: Option<SE3>) -> Matrix6xX<f64> {
        // let mut jac = Matrix6xX::<f64>::zeros(self.ndof());
        // Initialize the current transformation to identity
        let jac = self.jacobian0(q);
        match tool {
            Some(t) => return t.adjoint() * jac,
            None => {}
        }
        jac
    }

    pub fn ik(
        &self,
        t: &SE3,
        qinit: Option<DVector<f64>>,
        base: &Option<SE3>,
        tool: &Option<SE3>,
        maxiter: Option<usize>,
        weights_vec: Option<Vector6<f64>>,
        tolerance: Option<f64>,
    ) -> (DVector<f64>, bool) {
        let tol = tolerance.unwrap_or(1e-6);
    
        let mut qk = match qinit {
            Some(qvec) => qvec,
            None => DVector::from_fn(self.ndof(), |_i, _| rand::random()),
        };
        let ee_qk = self.fk(&qk, base, tool);
        println!("Initial Guess (End-effector):\n{:.3}", ee_qk.matrix());
        // println!("ee_qk:\n{:.3}", ee_qk.matrix());
    
        let we = match weights_vec {
            Some(vec) => Matrix6::from_diagonal(&vec),
            None => Matrix6::<f64>::identity()
        };
    
        for k in 0..maxiter.unwrap_or(50usize) {
            // Compute forward kinematics
            let tk = self.fk(&qk, &base, &tool);
    
            // Compute error using angle_axis
            let err = match base {
                Some(transform) => angle_axis(&(&transform.clone().inv() * &tk), &(&transform.clone().inv() * t)),
                None => angle_axis(&tk, &t),
            }; 
            // let err = angle_axis(&(&base.inv() * &tk), &(&base.inv() * T));
    
            // Compute the objective function: 0.5 * err^T * We * err
            let error_norm = 0.5 * (err.transpose() * we * err)[(0, 0)];
            // println!(" iter {}, error norm = {}", k, error_norm);
    
            // Check stopping condition
            if error_norm < tol {
                qk = qk.map(|v| v.sin().atan2(v.cos()));
                // println!("Success with {} iterations!", k);
                return (qk, true);
            }
    
            // Compute Jacobian
            // let jacobian = self.jacobian(&qk, None);
            let jacobian = self.jacobian0(&qk);
    
            // Compute the gradient: g = jacobian^T * We * err
            let g = jacobian.transpose() * we * err;
    
            // Compute the step size (damping factor)
            let lambda = 0.1;
            let wn = lambda * error_norm * DMatrix::<f64>::identity(self.ndof(), self.ndof());
    
            // Compute the update step: dq = (jacobian^T * We * jacobian + Wn)^-1 * g
            let jt_we_j_plus_wn = jacobian.transpose() * we * jacobian + wn;
            let dq = match jt_we_j_plus_wn.try_inverse() {
                Some(m) => m * g,
                None => panic!("No Inverse!"),
            };
            // let dq = m_inv * g;
    
            // Update joint configuration
            qk += dq;
        }
    
        // Return the final joint configuration and success flag
        // Apply angle normalization to keep qk within the valid range (-PI, PI)
        qk = qk.map(|v| v.sin().atan2(v.cos()));
        (qk, false)
    }
}

pub fn angle_axis(t: &SE3, tdesired: &SE3) -> Vector6<f64> {
    let mut residual = Vector6::zeros();
    // translation error
    let translation_residual = tdesired.translation() - t.translation();
    // rotation error
    let mut rotation_residual_vec = Vector3::zeros();
    let rotation_residual_matrix = tdesired.rotation_matrix() * t.rotation_matrix().transpose();
    let var = Vector3::from_vec(vec![
        rotation_residual_matrix[(2,1)] - rotation_residual_matrix[(1,2)],
        rotation_residual_matrix[(0,2)] - rotation_residual_matrix[(2,0)],
        rotation_residual_matrix[(1,0)] - rotation_residual_matrix[(0,1)],
    ]);
    let rotation_residual_matrix_trace = rotation_residual_matrix.trace();
    let var_norm = var.norm();
    
    if var_norm < 1e-6 {
        // todo!();
        if rotation_residual_matrix_trace > 0f64 {
            // keep zeros for `rotation_residual_vec`
            // rotation_residual_vec = Vector3::zeros();
            return residual;
        } else {
            // rotation_residual_vec = FRAC_PI_2 * ()
            rotation_residual_vec = Vector3::from_vec(vec![
                FRAC_PI_2 * (rotation_residual_matrix[(0,0)] + 1f64),
                FRAC_PI_2 * (rotation_residual_matrix[(1,1)] + 1f64),
                FRAC_PI_2 * (rotation_residual_matrix[(2,2)] + 1f64)
            ])
        }
    } else {
        let angle = var_norm.atan2(rotation_residual_matrix_trace - 1f64);
        rotation_residual_vec = (angle / var_norm) * var; 
    }

    residual.fixed_rows_mut::<3>(3).copy_from(&rotation_residual_vec);
    residual.fixed_rows_mut::<3>(0).copy_from(&translation_residual);
    residual
}

impl std::fmt::Display for ETS {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        // Use the iterator to join the formatted ET elements with a delimiter (e.g., ", ")
        let sequence_str = self
            .sequence
            .iter()
            .map(|et| format!("{}", et)) // Use Display implementation of ET
            .collect::<Vec<String>>()
            .join("\n ⊕ "); // Join the elements with a comma and space

        write!(f, "[{}]", sequence_str) // Write the result in the format "[...]" for ETS
    }
}
