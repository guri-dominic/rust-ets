extern crate nalgebra as na;
use na::{DMatrix, DVector};
use std::collections::HashSet;

/// Compute the RREF (Reduced Row Echelon Form) of a DMatrix<f64> and return pivot columns
pub fn rref(reference_matrix: &DMatrix<f64>) -> (DMatrix<f64>, HashSet<usize>) {
    let mut matrix = reference_matrix.clone();
    let (nrows, ncols) = matrix.shape();
    let mut lead = 0;
    let mut pivots = HashSet::new();

    for r in 0..nrows {
        if lead >= ncols {
            return (matrix.clone(), pivots);
        }

        let mut i = r;
        while matrix[(i, lead)] == 0.0 {
            i += 1;
            if i == nrows {
                i = r;
                lead += 1;
                if lead == ncols {
                    return (matrix.clone(), pivots);
                }
            }
        }

        // Swap rows if necessary
        if i != r {
            matrix.swap_rows(i, r);
        }

        // Normalize the row by dividing by the leading value
        let lv = matrix[(r, lead)];
        for j in 0..ncols {
            matrix[(r, j)] /= lv;
        }

        // Make other rows zero in the current column
        for i in 0..nrows {
            if i != r {
                let scale = matrix[(i, lead)];
                for j in 0..ncols {
                    matrix[(i, j)] -= scale * matrix[(r, j)];
                }
            }
        }

        // Store the pivot column index
        pivots.insert(lead);
        lead += 1;
    }

    (matrix.clone(), pivots)
}

/// Compute the null space of a matrix using RREF
pub fn null_space(matrix: &DMatrix<f64>) -> Vec<DVector<f64>> {
    let (nrows, ncols) = matrix.shape();
    let mut matrix = matrix.clone();

    // Compute RREF and pivots
    let (rref_matrix, pivots) = rref(&mut matrix);

    // Identify free variables (columns that are not pivot columns)
    let free_vars: Vec<usize> = (0..ncols).filter(|j| !pivots.contains(j)).collect();

    // Construct the null space basis vectors
    let mut null_space_basis = Vec::new();
    for &free_var in &free_vars {
        // Initialize the null vector with zeros
        let mut null_vector = DVector::zeros(ncols);
        null_vector[free_var] = 1.0;

        // Solve for the dependent variables (pivot columns)
        for _i in 0..nrows {
            if pivots.contains(&free_var) {
                continue;
            }
            for &pivot_col in &pivots {
                let pivot_row = (0..nrows).find(|&r| rref_matrix[(r, pivot_col)] == 1.0);
                if let Some(row_idx) = pivot_row {
                    null_vector[pivot_col] = -rref_matrix[(row_idx, free_var)];
                }
            }
        }

        // Add the null vector to the basis
        // null_vector.normalize_mut();
        null_space_basis.push(null_vector);
    }

    null_space_basis
}