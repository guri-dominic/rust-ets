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

    match parse_ets_from_string(&ets_string) {
        Ok(ets) => {
            println!("ets: {}", ets);
            println!("ndof: {}", ets.ndof());
            println!("joints: {:?}", ets.joints_ets_indices());
        }
        Err(e) => println!("{}", e),
    }
}
