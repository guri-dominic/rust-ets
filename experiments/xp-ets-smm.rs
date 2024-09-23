// use ets::{ETS, SE3};
use nalgebra::{DMatrix, DVector, Matrix, Matrix6xX};

use ets::ETS::ETS;

// Compute the RREF (Reduced Row Echelon Form) of a matrix.
fn rref(matrix: &Matrix6xX<f64>) -> (Matrix6xX<f64>, Vec<usize>) {
    let mut m = matrix.clone();
    let (nrows, ncols) = m.shape();
    let mut lead = 0usize;

    let mut pivot_columns = Vec::new();
    let mut free_columns = Vec::new();
    for r in 0..nrows {
        pivot_columns.push(r);
        free_columns.push(r);
        if lead >= ncols {
            return (m, pivot_columns);
        }
        let mut i = r;
        while m[(i,lead)].abs() < 1e-6 {
            i += 1usize;
            if i == nrows {
                i = r;
                lead += 1;
                if lead == ncols {
                    return (m, pivot_columns);
                }
            }
        }
        if i != r {
            m.swap_rows(i, r);
        }
        let lead_value = m[(r, lead)];
        let scale = 1f64 / lead_value;
        // scale row r
        for j in 0..(7usize) {
            m[(r, j)] *= scale;
        }
        for i in 0..nrows {
            if i != r {
                let lead_value = m[(i, lead)];
                for j in 0..ncols {
                    m[(i, j)] -= lead_value * m[(r, j)];
                }
            }
        }
        pivot_columns.push(lead);
        lead += 1;
    }
    (m, pivot_columns)
}


fn null_space(matrix: &Matrix6xX<f64>) -> Vec<DVector<f64>> {
    let (rref_matrix, pivot_columns) = rref(matrix);
    let (nrows, ncols) = rref_matrix.shape();

    // Identify free variables (columns without leading 1)
    // let mut pivot_columns = Vec::new();
    let free_columns: Vec<&usize> = pivot_columns.iter().filter(|x| !pivot_columns.contains(x)).collect();
    // free_vars = [j for j in range(ncols) if j not in pivots]
    println!("pivot columns = {pivot_columns:?}");
    println!(" free columns = {free_columns:?}");

    // Build the null space basis vectors
    let mut null_space_vectors = Vec::new();
    for &free_col in &free_columns {
        let mut null_vector = DVector::zeros(ncols);
        null_vector[*free_col] = 1.0;
        for i in 0..nrows {
            if pivot_columns.contains(&free_col) {
                continue;
            }
            if pivot_columns.contains(&i) {
                let pivot_col = pivot_columns[i];
                // null_vector_list[pivot_col] = (-1*rref_matrix[i,free_var].to_expression())
                null_vector[pivot_col] = -rref_matrix[(i,*free_col)];
            }
        }
        // for &pivot_col in &pivot_columns {
        //     let pivot_row = (0..rows).find(|&i| (rref_matrix[(i, pivot_col)] - 1.0).abs() < 1e-6).unwrap();
        //     null_vector[pivot_col] = -rref_matrix[(pivot_row, *free_col)];
        // }
        null_space_vectors.push(null_vector);
    }

    null_space_vectors
}


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
    println!("Jacobian: \n{:.8}", jacobian.clone());

    // let kernel = null_space(&jacobian);
    // for (k, v) in kernel.iter().enumerate() {
    //     println!(" > {} ker(Jacobian): \n{:.5}", k, v.transpose());
    // }

    let (m_rref, pivots) = rref(&jacobian);
    println!("rref(Jacobian): \n{:.8}", m_rref);

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
