use std::f64::consts::FRAC_PI_2;

use ets::{ETS, SE3};
use nalgebra::DVector;

fn main() {
    let ets = ETS::xarm7();
    println!("ets: {}", ets);
    println!("ndof: {}", ets.ndof());
    println!("joints: {:?}", ets.joints_ets_indices());

    let q = DVector::from_fn(7, |_i, _| 0f64);
    // Compute forward kinematics
    let end_effector = ets.fk(&q, &None, &None);
    println!("End-effector transformation:\n{:.3}", end_effector.matrix());

    // Compute the Jacobian
    let jacobian = ets.jacobian0(&q);
    println!("Jacobian: \n{:.8}", jacobian);

    let ee = SE3::xyzrpy(0.35, 0f64, 0.45, 0f64, -FRAC_PI_2, 0f64);

    let base = Some(SE3::tz(0.12));
    let tool = Some(SE3::new());
    // Define initial guess for q0
    let qinit = None;
    
    // Calling the IK solver
    // let (qk, success) = ik(&ets, &ee, q0, &base, &tool, 50, None, 1e-6);
    let (qk, _success) = ets.ik(&ee, qinit, &base, &tool, None, None, None);
    let ee_qk = ets.fk(&qk, &base, &None);
    println!("     IK (target):{:.3}", ee.matrix());
    println!("   IK (solution):{:.5}", qk.transpose());
    println!("ee_qk (solution):{:.3}", ee_qk.matrix());
}
