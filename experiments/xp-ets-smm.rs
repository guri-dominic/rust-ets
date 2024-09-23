// use ets::{ETS, SE3};
use nalgebra::DVector;

use ets::ets::ETS;

use ets::linear_algebra::{rref, null_space};

fn main() {
    let ets = ETS::xarm7();
    println!("ets: {}", ets);
    println!("ndof: {}", ets.ndof());
    println!("joints: {:?}", ets.joints_ets_indices());

    let q = DVector::from_fn(7, |_i, _| 0f64);
    // Compute forward kinematics
    let end_effector = ets.fk(&q, &None, &None);
    println!("End-effector transformation:\n{:.3}", end_effector.matrix());

    let (q, _success) = ets.ik(&end_effector, None, &None, &None, None, None, None);

    // Compute the Jacobian
    let jacobian = ets.jacobian0(&q);
    println!("Jacobian: \n{:.8}", jacobian.clone());

    // let kernel = null_space(&jacobian);
    // for (k, v) in kernel.iter().enumerate() {
    //     println!(" > {} ker(Jacobian): \n{:.5}", k, v.transpose());
    // }

    let (m_rref, pivots) = rref(&jacobian);
    println!("rref(Jacobian).matrix: \n{:.8}", m_rref);
    println!("rref(Jacobian).pivots: \n{:?}", pivots);

    println!("=====================================================================");
    let null = null_space(&jacobian);
    for (k, v) in null.iter().enumerate() {
        println!("null vector ({k}) = {v:.3}");
    }

    // let mut jac = jacobian;
    // println!("Jacobian: \n{:.8}", jac.clone());
    


    // let ee = SE3::xyzrpy(0.35, 0f64, 0.45, 0f64, -FRAC_PI_2, 0f64);
    // let base = Some(SE3::tz(0.12));
    // let tool = Some(SE3::new());
    // let qinit = None;
    
    //// Calling the IK solver
    // let (qk, _success) = ets.ik(&ee, qinit, &base, &tool, None, None, None);
    // let ee_qk = ets.fk(&qk, &base, &None);
    // println!("     IK (target):{:.3}", ee.matrix());
    // println!("   IK (solution):{:.5}", qk.transpose());
    // println!("ee_qk (solution):{:.3}", ee_qk.matrix());
}
