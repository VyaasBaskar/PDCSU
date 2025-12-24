import numpy as np

# Simulated system parameters
J_true = 0.05       # kg·m²
b_true = 0.01       # N·m·s/rad
k_motor = 0.1       # torque per duty cycle unit
dt = 0.001          # 1 ms loop

# Generate a simple duty cycle profile (sinusoidal)
time = np.arange(0, 2, dt)
duty_cycle_stream = 0.5 * np.sin(2 * np.pi * 0.5 * time) + 0.5  # 0..1 PWM
velocity_stream = np.zeros_like(time)
omega = 0.0

# Simulate the system
for i, duty in enumerate(duty_cycle_stream):
    torque = k_motor * duty
    omega_dot = (torque - b_true * omega) / J_true
    omega += omega_dot * dt
    velocity_stream[i] = omega

# RLS estimator (from previous snippet)
class InertiaEstimator:
    def __init__(self, forgetting_factor=0.99):
        self.theta = np.array([0.01, 0.01])
        self.P = np.eye(2) * 1000
        self.lambda_ = forgetting_factor
        self.prev_vel = 0.0
        self.dt = dt

    def update(self, duty_cycle, velocity):
        omega_dot = (velocity - self.prev_vel) / self.dt
        self.prev_vel = velocity
        phi = np.array([duty_cycle * k_motor, -velocity])
        P_phi = self.P @ phi
        gain = P_phi / (self.lambda_ + phi.T @ P_phi)
        error = omega_dot - phi.T @ self.theta
        self.theta += gain * error
        self.P = (self.P - np.outer(gain, P_phi)) / self.lambda_
        k_over_J, b_over_J = self.theta
        J_est = 1.0 / k_over_J if k_over_J != 0 else np.inf
        b_est = b_over_J * J_est
        return J_est, b_est

estimator = InertiaEstimator(forgetting_factor=0.995)

# Run estimator on simulated data
J_estimates = []
b_estimates = []

for duty, vel in zip(duty_cycle_stream, velocity_stream):
    J, b = estimator.update(duty, vel)
    J_estimates.append(J)
    b_estimates.append(b)

# Print final estimates
print(f"Final estimated inertia: {J_estimates[-1]:.5f}, friction: {b_estimates[-1]:.5f}")
