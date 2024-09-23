use nalgebra::{DMatrix, DVector, Matrix4, Matrix6, Vector6};

use crate::se3::{angle_axis, SE3};

// use lazy_static::lazy_static;
// use std::collections::HashMap;

// lazy_static! {
//     static ref MODELS: HashMap<str, &'str> = {
//         let mut m = HashMap::new();
//         m.insert("xarm7", "SE3() ⊕ SE3(0, 0, 0.267) ⊕ Rz(q0) ⊕ SE3(-90°, -0°, 0°) ⊕ Rz(q1) ⊕ SE3(0, -0.293, 0; 90°, -0°, 0°) ⊕ Rz(q2) ⊕ SE3(0.0525, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3) ⊕ SE3(0.0775, -0.3425, 0; 90°, -0°, 0°) ⊕ Rz(q4) ⊕ SE3(90°, -0°, 0°) ⊕ Rz(q5) ⊕ SE3(0.076, 0.097, 0; -90°, -0°, 0°) ⊕ Rz(q6) ⊕ SE3() ⊕ SE3()");
//         m.insert("panda", "SE3(0, 0, 0.333) ⊕ Rz(q0) ⊕ SE3(-90°, -0°, 0°) ⊕ Rz(q1) ⊕ SE3(0, -0.316, 0; 90°, -0°, 0°) ⊕ Rz(q2) ⊕ SE3(0.0825, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3) ⊕ SE3(-0.0825, 0.384, 0; -90°, -0°, 0°) ⊕ Rz(q4) ⊕ SE3(90°, -0°, 0°) ⊕ Rz(q5) ⊕ SE3(0.088, 0, 0; 90°, -0°, 0°) ⊕ Rz(q6) ⊕ SE3(0, 0, 0.107) ⊕ SE3(0°, -0°, -45°) ⊕ SE3(0, 0, 0.1034)");
//         m
//     };
//     static ref MODELS_TOTALS: usize = HASHMAP.len();
// }

const XARM7:&str = "SE3() ⊕ SE3(0, 0, 0.267) ⊕ Rz(q0) ⊕ SE3(-90°, -0°, 0°) ⊕ Rz(q1) ⊕ SE3(0, -0.293, 0; 90°, -0°, 0°) ⊕ Rz(q2) ⊕ SE3(0.0525, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3) ⊕ SE3(0.0775, -0.3425, 0; 90°, -0°, 0°) ⊕ Rz(q4) ⊕ SE3(90°, -0°, 0°) ⊕ Rz(q5) ⊕ SE3(0.076, 0.097, 0; -90°, -0°, 0°) ⊕ Rz(q6) ⊕ SE3() ⊕ SE3()";
const PANDA:&str = "SE3(0, 0, 0.333) ⊕ Rz(q0) ⊕ SE3(-90°, -0°, 0°) ⊕ Rz(q1) ⊕ SE3(0, -0.316, 0; 90°, -0°, 0°) ⊕ Rz(q2) ⊕ SE3(0.0825, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3) ⊕ SE3(-0.0825, 0.384, 0; -90°, -0°, 0°) ⊕ Rz(q4) ⊕ SE3(90°, -0°, 0°) ⊕ Rz(q5) ⊕ SE3(0.088, 0, 0; 90°, -0°, 0°) ⊕ Rz(q6) ⊕ SE3(0, 0, 0.107) ⊕ SE3(0°, -0°, -45°) ⊕ SE3(0, 0, 0.1034)";


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
                let roll_deg = roll.to_degrees();
                let pitch_deg = pitch.to_degrees();
                let yaw_deg = yaw.to_degrees();
                write!(f, "SE3({x},{y},{z};{roll_deg}°,{pitch_deg}°,{yaw_deg}°)")
            }
        }
    }
}

pub struct ETS {
    pub sequence: Vec<ET>,
}

impl ETS {
    pub fn new() -> Self {
        ETS { sequence: vec![] }
    }

    pub fn from_vec(sequence: Vec<ET>) -> Self {
        ETS { sequence }
    }

    // pub fn model(version: str) -> Self {
    //     match MODELS.get(&version) {
    //     }
    // }

    pub fn xarm7() -> Self {
        match parse_ets_from_string(XARM7) {
            Ok(ets) => {
                return ets;
            }
            Err(e) => panic!("Parsing 'XARM7' failed: {}", e),
        }
    }

    pub fn panda() -> Self {
        match parse_ets_from_string(PANDA) {
            Ok(ets) => {
                return ets;
            }
            Err(e) => panic!("Parsing 'PANDA' failed: {}", e),
        }
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

    pub fn joint_jacobian(&self, _joint_index: usize, _q: &DVector<f64>) -> Matrix4<f64> {
        let dt = Matrix4::<f64>::identity();
        dt
    }

    // pub fn jacobian_rotational_component(&self, joint_index: usize, q: &DVector<f64>, base: Option<SE3>, tool: Option<SE3>) -> Vector3<f64> {
    //     // let mut jacobian = Matrix3xX::zeros(self.ndof());
    //     let end_effector = self.fk(q, base, tool);
    //     vex(rot3(self.joint_jacobian(joint_index, q)) * rot3(end_effector.matrix()).transpose())
    // }

    pub fn jacobian0(&self, q: &DVector<f64>) -> DMatrix<f64> {
        // let mut jacobian_matrix = Matrix6xX::<f64>::zeros(self.ndof());
        let mut jacobian_matrix = DMatrix::zeros(6, self.ndof());
        let end_effector = self.fk(q, &None, &None);
        let mut current_frame = SE3::from_matrix(Matrix4::identity());
        for (_k, et) in self.sequence.iter().enumerate() {
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
                    let x = distal_transform.matrix()[(0, 3)];
                    let y = distal_transform.matrix()[(1, 3)];
                    // let z = distal_transform.matrix()[(2,3)];
                    let n = current_frame.matrix().fixed_view::<3, 1>(0, 0).into_owned();
                    let o = current_frame.matrix().fixed_view::<3, 1>(0, 1).into_owned();
                    let a = current_frame.matrix().fixed_view::<3, 1>(0, 2).into_owned();
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

    pub fn jacobian(&self, q: &DVector<f64>, tool: Option<SE3>) -> DMatrix<f64> {
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
            None => Matrix6::<f64>::identity(),
        };

        for _k in 0..maxiter.unwrap_or(50usize) {
            // Compute forward kinematics
            let tk = self.fk(&qk, &base, &tool);

            // Compute error using angle_axis
            let err: Vector6<f64> = match base {
                Some(transform) => angle_axis(
                    &(&transform.clone().inv() * &tk),
                    &(&transform.clone().inv() * t),
                ),
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
            let weighted_error = we * err;
            let g = jacobian.transpose() * weighted_error;

            // Compute the step size (damping factor)
            let lambda = 0.1;
            let wn = lambda * error_norm * DMatrix::<f64>::identity(self.ndof(), self.ndof());

            // Compute the update step: dq = (jacobian^T * We * jacobian + Wn)^-1 * g
            let jt_we_j_plus_wn = jacobian.transpose() * we * jacobian + wn;
            let dq: DVector<_> = match jt_we_j_plus_wn.try_inverse() {
                Some(m) => m * g,
                None => panic!("No Inverse!"),
            };
            // let dq = m_inv * g;

            // Update joint configuration
            qk = qk + dq;
        }

        // Return the final joint configuration and success flag
        // Apply angle normalization to keep qk within the valid range (-PI, PI)
        qk = qk.map(|v| v.sin().atan2(v.cos()));
        (qk, false)
    }
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

fn parse_transform(transformation: &str) -> Result<ET, String> {
    // element.split_inclusive("(")
    let open_paren = transformation
        .find('(')
        .ok_or("Missing opening parenthesis");
    let open_index = match open_paren {
        Ok(index) => index,
        Err(_error) => return Err(String::from("Invalid openning paranthesis index!")),
    };
    let close_paren = transformation
        .find(')')
        .ok_or("Missing closing parenthesis");
    let close_index = match close_paren {
        Ok(index) => index,
        Err(_error) => return Err(String::from("Invalid closing paranthesis index!")),
    };
    //Extract the type and the value
    let transform_type_str = &transformation[0..open_index];
    let value_str = &transformation[open_index + 1..close_index];
    let _transform = match (transform_type_str, value_str) {
        ("Rz", arg_index) => {
            // let joint_index = args[];
            if let Some(remainder) = arg_index.strip_prefix('q') {
                // Try to parse the remainder as i32
                match remainder.parse::<i32>() {
                    Ok(num) => {
                        return Ok(ET::Rz {
                            angle: 0f64,
                            isjoint: true,
                            index: num,
                        });
                    }
                    Err(_) => return Err(String::from("Joint Index failed to parse as i32")),
                }
            } else {
                return Err(String::from("The input does not start with 'q'"));
            }
        }
        ("SE3", args) => {
            if args.len() > 0usize {
                if args.contains(";") {
                    let parts: Vec<&str> = args.split(';').collect();
                    if parts.len() != 2 {
                        return Err(String::from("Input format is incorrect"));
                    }

                    // First part: Parse the Cartesian coordinates
                    let cartesian_coords: Vec<f64> = parts[0]
                        .split(',')
                        .map(|s| s.parse::<f64>().unwrap())
                        .collect();

                    // Second part: Parse the angles, remove '°', and convert to radians
                    let angles_in_radians: Vec<f64> = parts[1]
                        .split(',')
                        .map(|s| s.trim_end_matches('°').parse::<f64>().unwrap().to_radians())
                        .collect();

                    let x = cartesian_coords[0];
                    let y = cartesian_coords[1];
                    let z = cartesian_coords[2];
                    let roll = angles_in_radians[0];
                    let pitch = angles_in_radians[1];
                    let yaw = angles_in_radians[2];
                    return Ok(ET::T {
                        x: x,
                        y: y,
                        z: z,
                        roll: roll,
                        pitch: pitch,
                        yaw: yaw,
                    });
                } else if args.contains('°') {
                    let angles_in_radians: Vec<f64> = args
                        .split(',')
                        .map(|s| s.trim_end_matches('°').parse::<f64>().unwrap().to_radians())
                        .collect();
                    let roll = angles_in_radians[0];
                    let pitch = angles_in_radians[1];
                    let yaw = angles_in_radians[2];
                    return Ok(ET::RPY {
                        roll: roll,
                        pitch: pitch,
                        yaw: yaw,
                    });
                } else {
                    let cartesian_coords: Vec<f64> =
                        args.split(',').map(|s| s.parse::<f64>().unwrap()).collect();
                    let x = cartesian_coords[0];
                    let y = cartesian_coords[1];
                    let z = cartesian_coords[2];
                    return Ok(ET::Transl { x: x, y: y, z: z });
                }
            }
        }
        _ => return Err(String::from("Invalid Transform String")),
    };
    // print!("{}", transform_type_str);
    // println!(": {}", value_str);
    Ok(ET::I)
}

pub fn parse_ets_from_string(xarm_7_ets_string: &str) -> Result<ETS, String> {
    // let transformations: Vec<&str> = transformations.split_whitespace().collect();
    let mut sequence = Vec::new();
    let no_space_string = xarm_7_ets_string.replace(" ", "");
    let string_transforms: Vec<&str> = no_space_string.split('⊕').collect();
    // let _ = string_transforms.map(|x| println!("{}", x));
    for transformation in &string_transforms {
        match parse_transform(&transformation) {
            Ok(et) => sequence.push(et),
            Err(e) => return Err(e),
        }
    }
    Ok(ETS::from_vec(sequence))
}
