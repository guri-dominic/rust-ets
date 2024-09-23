use nalgebra::{Vector3,Vector6,Matrix3,Matrix4};



pub fn skew(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

pub(crate) fn skew_twist(twist: Vector6<f64>) -> Matrix4<f64> {
    // Split the twist vector into angular (omega) and linear (v) components
    let omega = Vector3::new(twist[3], twist[4], twist[5]); // Angular part
    let v = Vector3::new(twist[0], twist[1], twist[2]); // Linear part

    // Construct the skew-symmetric matrix for the twist
    Matrix4::new(
        0.0, -omega.z, omega.y, v.x, omega.z, 0.0, -omega.x, v.y, -omega.y, omega.x, 0.0, v.z, 0.0,
        0.0, 0.0, 0.0,
    )
}

pub fn rot3(m: Matrix4<f64>) -> Matrix3<f64> {
    return m.fixed_view::<3, 3>(0, 0).into();
}

pub fn vex(m: Matrix3<f64>) -> Vector3<f64> {
    // Extract the angular velocity components from the skew-symmetric 3x3 submatrix
    let omega_x = m[(2, 1)]; // element (2,1) is -ω_x, so take it directly
    let omega_y = m[(0, 2)]; // element (0,2) is ω_y
    let omega_z = m[(1, 0)]; // element (1,0) is ω_z
                             // Combine into a twist vector: [v_x, v_y, v_z, omega_x, omega_y, omega_z]
    Vector3::new(omega_x, omega_y, omega_z)
}

pub fn mat_to_twist(m: Matrix4<f64>) -> Vector6<f64> {
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
