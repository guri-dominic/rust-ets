use ets::parse_ets_from_string;

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
