use std::f64::consts::FRAC_PI_2;

use ets::{ET, ETS, SE3};
use nalgebra::DVector;

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

fn parse_ets_from_string(xarm_7_ets_string: &str) -> Result<ETS, String> {
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
    Ok(ETS::new(sequence))
}

fn main() {
    // let xarm_7_ets_string = String::from("SE3() ⊕ SE3(0, 0, 0.267) ⊕ Rz(q0) ⊕ SE3(-90°, -0°, 0°) ⊕ Rz(q1) ⊕ SE3(0, -0.293, 0; 90°, -0°, 0°) ⊕ Rz(q2) ⊕ SE3(0.0525, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3) ⊕ SE3(0.0775, -0.3425, 0; 90°, -0°, 0°) ⊕ Rz(q4) ⊕ SE3(90°, -0°, 0°) ⊕ Rz(q5) ⊕ SE3(0.076, 0.097, 0; -90°, -0°, 0°) ⊕ Rz(q6) ⊕ SE3() ⊕ SE3()");
    // let panda_ets_string = String::from("SE3(0, 0, 0.333) ⊕ Rz(q0) ⊕ SE3(-90°, -0°, 0°) ⊕ Rz(q1) ⊕ SE3(0, -0.316, 0; 90°, -0°, 0°) ⊕ Rz(q2) ⊕ SE3(0.0825, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3) ⊕ SE3(-0.0825, 0.384, 0; -90°, -0°, 0°) ⊕ Rz(q4) ⊕ SE3(90°, -0°, 0°) ⊕ Rz(q5) ⊕ SE3(0.088, 0, 0; 90°, -0°, 0°) ⊕ Rz(q6) ⊕ SE3(0, 0, 0.107) ⊕ SE3(0°, -0°, -45°) ⊕ SE3(0, 0, 0.1034)");
    // let ets_string = String::from("SE3() ⊕ SE3(0, 0, 0.267) ⊕ Rz(q0) ⊕ SE3(-90°, -0°, 0°) ⊕ Rz(q1) ⊕ SE3(0, -0.293, 0; 90°, -0°, 0°) ⊕ Rz(q2) ⊕ SE3(0.0525, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3) ⊕ SE3(0.0775, -0.3425, 0; 90°, -0°, 0°) ⊕ Rz(q4) ⊕ SE3(90°, -0°, 0°) ⊕ Rz(q5) ⊕ SE3(0.076, 0.097, 0; -90°, -0°, 0°) ⊕ Rz(q6) ⊕ SE3() ⊕ SE3()");
    let ets_string = String::from("SE3(0, 0, 0.333) ⊕ Rz(q0) ⊕ SE3(-90°, -0°, 0°) ⊕ Rz(q1) ⊕ SE3(0, -0.316, 0; 90°, -0°, 0°) ⊕ Rz(q2) ⊕ SE3(0.0825, 0, 0; 90°, -0°, 0°) ⊕ Rz(q3) ⊕ SE3(-0.0825, 0.384, 0; -90°, -0°, 0°) ⊕ Rz(q4) ⊕ SE3(90°, -0°, 0°) ⊕ Rz(q5) ⊕ SE3(0.088, 0, 0; 90°, -0°, 0°) ⊕ Rz(q6) ⊕ SE3(0, 0, 0.107) ⊕ SE3(0°, -0°, -45°) ⊕ SE3(0, 0, 0.1034)");

    // // println!("x7 ETS: {}", xarm_7_ets_string);
    // let no_space_string = xarm_7_ets_string.replace(" ", "");
    // let string_transforms: Vec<&str> = no_space_string.split('⊕').collect();
    // // let _ = string_transforms.map(|x| println!("{}", x));
    // for transformation in &string_transforms {
    //     let transform = parse_transform(&transformation);
    //     match transform {
    //         Ok(et) => println!("{}", et),
    //         Err(e) => println!("{}", e),
    //     }
    // }

    match parse_ets_from_string(&ets_string) {
        Ok(ets) => {
            println!("ets: {}", ets);
            println!("ndof: {}", ets.ndof());
            println!("joints: {:?}", ets.joints_ets_indices());
            // Joint values (for the joint transformation at index 2, assuming the second Rz is a joint)
            // let q = vec![45.0_f64.to_radians()];
            // let q = DVector::from_vec(vec![0f64, 0f64, 0f64, 0f64, 0f64, 0f64, 0f64]);
            // let q = DVector::from_vec(vec![1f64, 1f64, 1f64, 1f64, 1f64, 1f64, 1f64]);

            /*
            let v = Vector3::from_fn(|i, _| i);
            // The additional argument represents the vector dimension.
            let dv = DVector::from_fn(3, |i, _| i);
            let m = Matrix2x3::from_fn(|i, j| i * 3 + j);
            // The two additional arguments represent the matrix dimensions.
            let dm = DMatrix::from_fn(2, 3, |i, j| i * 3 + j);
            */

            let q = DVector::from_fn(7, |_i, _| 0f64);
            // Compute forward kinematics
            let end_effector = ets.fk(&q, &None, &None);
            println!("End-effector transformation:\n{:.3}", end_effector.matrix());

            // Compute the Jacobian
            // let tool = SE3::tz(0.12f64);
            // let jacobian = ets.jacobian0(&q, std::option::Option::Some(tool));
            let jacobian = ets.jacobian0(&q);
            println!("Jacobian: \n{:.8}", jacobian);

            let ee = SE3::xyzrpy(0.35, 0f64, 0.45, 0f64, -FRAC_PI_2, 0f64);

            let base = Some(SE3::new());
            let tool = Some(SE3::new());
            // Define initial guess for q0
            let qinit = None;
            // Call the IK solver
            // let (qk, success) = ik(&ets, &ee, q0, &base, &tool, 50, None, 1e-6);
            let (qk, _success) = ets.ik(&ee, qinit, &base, &tool, None, None, None);
            let ee_qk = ets.fk(&qk, &None, &None);
            println!("IK (target):\n{:.3}", ee.matrix());
            println!("IK (solution): {:.5}", qk.transpose());
            println!("ee_qk:\n{:.3}", ee_qk.matrix());
        }
        Err(e) => println!("{}", e),
    }
}

/*
fn ik(
    ets: &ETS,
    T: &SE3,
    q0: Option<DVector<f64>>,
    base: &SE3,
    tool: &SE3,
    maxiter: usize,
    weights_vec: Option<Vector6<f64>>,
    tol: f64,
) -> (DVector<f64>, bool) {
    // let mut qk = if let Some(initial_q) = q0 {
    //     initial_q
    // } else {
    //     DVector::from_fn(ets.ndof(), |_i, _| rand::random())
    // };

    let mut qk = match q0 {
        Some(qvec) => qvec,
        None => DVector::from_fn(ets.ndof(), |_i, _| rand::random()),
    };
    let ee_qk = ets.fk(&qk, None, None);
    println!("Initial Guess (End-effector):\n{:.3}", ee_qk.matrix());
    // println!("ee_qk:\n{:.3}", ee_qk.matrix());

    let we = match weights_vec {
        Some(vec) => Matrix6::from_diagonal(&vec),
        None => Matrix6::<f64>::identity()
    };

    for k in 0..maxiter {
        // Compute forward kinematics
        let tk = ets.fk(&qk, None, None);

        // Compute error using angle_axis
        let err = angle_axis(&(&base.inv() * &tk), &(&base.inv() * T));

        // Compute the objective function: 0.5 * err^T * We * err
        let error_norm = 0.5 * (err.transpose() * we * err)[(0, 0)];
        println!(" iter {}, error norm = {}", k, error_norm);

        // Check stopping condition
        if error_norm < 1e-6 {
            qk = qk.map(|v| v.sin().atan2(v.cos()));
            println!("Success with {} iterations!", k);
            return (qk, true);
        }

        // Compute Jacobian
        // let jacobian = ets.jacobian(&qk, None);
        let jacobian = ets.jacobian0(&qk);

        // Compute the gradient: g = jacobian^T * We * err
        let g = jacobian.transpose() * we * err;

        // Compute the step size (damping factor)
        let lambda = 0.1;
        let wn = lambda * error_norm * DMatrix::<f64>::identity(ets.ndof(), ets.ndof());

        // Compute the update step: dq = (jacobian^T * We * jacobian + Wn)^-1 * g
        let jt_we_j_plus_wn = jacobian.transpose() * we * jacobian + wn;
        // println!("JtWeJ = {:.5}", JtWeJ);
        // println!("Wn = {:.5}", Wn);
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
*/