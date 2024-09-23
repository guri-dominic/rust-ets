use ets::linear_algebra::{rref, null_space};
use nalgebra::DMatrix;

fn main() {
    // Example matrix (3x4)
    let mut matrix = DMatrix::from_row_slice(3, 4, &[
        // 1.0, 2.0, 3.0, 4.0,
        // 2.0, 4.0, 6.0, 8.0,
        // 3.0, 6.0, 9.0, 12.0,
        1.0, 2.0, -1.0, -4.0,
        2.0, 3.0, -1.0, -11.0,
        -2.0, 0.0, -3.0, 22.0,
    ]);

    // Compute the RREF
    let (rref_matrix, pivots) = rref(&mut matrix);

    // Output the RREF matrix and pivot positions
    println!("RREF Matrix:\n{}", rref_matrix);
    println!("Pivot Columns: {:?}", pivots);

    // Compute the null space
    let null_space_basis = null_space(&matrix);

    // Print the null space basis vectors
    println!("Null space basis vectors:");
    for vec in null_space_basis {
        println!("{}", vec);
    }
}

