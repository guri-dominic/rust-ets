extern crate nalgebra as na;

use na::DVector;

fn euler<F>(f: F, y0: DVector<f64>, t0: f64, tf: f64, dt: f64) -> Vec<DVector<f64>>
where
    F: Fn(f64, &DVector<f64>) -> DVector<f64>,
{
    let mut y = y0;
    let mut t = t0;
    let mut result = Vec::new();

    while t < tf {
        result.push(y.clone());
        y += f(t, &y) * dt;
        t += dt;
    }

    result
}

impl System<f64> for ExponentialDecay {
    fn system(&self, _t: f64, y: &[f64], dydx: &mut [f64]) {
        dydx[0] = -y[0]; // dy/dt = -y (exponential decay)
    }
}

fn main() {
    // Initial condition
    let y0 = vec![1.0];

    // Time span
    let t0 = 0.0;
    let tf = 5.0;

    // Create the solver with Dormand-Prince (RK45)
    let mut solver = Dopri5::new(ExponentialDecay, t0, tf, 1e-6, y0, 0.01);

    // Solve the ODE
    let result = solver.integrate();

    match result {
        Ok(stats) => {
            let times = linspace(t0, tf, 100);
            let solutions = solver.interpolate(&times).unwrap();
            for (t, sol) in times.iter().zip(solutions.iter()) {
                println!("t: {}, solution: {}", t, sol[0]);
            }
            println!("Solver stats: {:?}", stats);
        }
        Err(e) => println!("An error occurred: {:?}", e),
    }
}