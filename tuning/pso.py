"""
Particle Swarm Optimization for wind speed parameter tuning.

Optimizes α, β, γ coefficients to minimize MSE between:
    v_estimated = α * √tan(θ) + β * θ̇ + γ
    v_ground_truth (measured)

This formula is derived from pendulum physics where drag force balances gravity.
"""

import json
import os
import numpy as np
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import List, Optional

from .parameters import WindParams, MAX_THETA

# PSO search bounds (adjusted for sqrt(tan) formula)
BOUNDS = {
    "alpha": (0.0, 50.0),   # Higher range for sqrt(tan) coefficient
    "beta": (-1.0, 1.0),
    "gamma": (-10.0, 10.0),
}

# PSO hyperparameters
N_PARTICLES = 30
N_ITERATIONS = 100
W = 0.7       # Inertia weight
C1 = 1.5      # Cognitive coefficient (personal best)
C2 = 1.5      # Social coefficient (global best)

# Data directory for saving tuning sessions
DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


@dataclass
class TuningDataPoint:
    """Single data point for tuning."""
    timestamp: float       # Relative time in seconds
    theta: float           # Angle from vertical (radians)
    theta_dot: float       # Angular velocity (rad/s)
    ground_truth: Optional[float]  # Ground truth wind speed (m/s), None if unavailable


def save_tuning_data(data: List[TuningDataPoint], has_ground_truth: bool) -> str:
    """
    Save tuning data to JSON file.

    Args:
        data: List of TuningDataPoint
        has_ground_truth: Whether ground truth was available

    Returns:
        Path to saved file
    """
    os.makedirs(DATA_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}.json"
    filepath = os.path.join(DATA_DIR, filename)

    save_data = {
        "timestamp": timestamp,
        "has_ground_truth": has_ground_truth,
        "n_samples": len(data),
        "data": [asdict(dp) for dp in data]
    }

    with open(filepath, 'w') as f:
        json.dump(save_data, f, indent=2)

    print(f"Tuning data saved to {filepath} ({len(data)} samples)")
    return filepath


def _compute_mse(params: np.ndarray, data: List[TuningDataPoint]) -> float:
    """
    Compute Mean Squared Error for given parameters.

    Uses formula: v = α * √tan(θ) + β * θ̇ + γ

    Args:
        params: Array [alpha, beta, gamma]
        data: List of TuningDataPoint with ground truth

    Returns:
        MSE value
    """
    alpha, beta, gamma = params
    errors = []

    for dp in data:
        if dp.ground_truth is not None:
            # Clamp theta to avoid tan(90°) = infinity
            theta_clamped = np.clip(dp.theta, 0, MAX_THETA)

            # Calculate sqrt(tan(theta))
            sqrt_tan_theta = np.sqrt(np.tan(theta_clamped))

            # Estimate wind speed
            estimated = alpha * sqrt_tan_theta + beta * dp.theta_dot + gamma
            estimated = max(0.0, estimated)  # Non-negative

            error = (estimated - dp.ground_truth) ** 2
            errors.append(error)

    if not errors:
        return float('inf')

    return np.mean(errors)


def optimize_parameters(data: List[TuningDataPoint]) -> Optional[WindParams]:
    """
    Run Particle Swarm Optimization to find best parameters.

    Args:
        data: List of TuningDataPoint with ground truth values

    Returns:
        Optimized WindParams, or None if optimization failed
    """
    # Filter to only data points with ground truth
    valid_data = [dp for dp in data if dp.ground_truth is not None]

    if len(valid_data) < 10:
        print(f"Not enough data for optimization ({len(valid_data)} valid samples)")
        return None

    print(f"Running PSO optimization with {len(valid_data)} samples...")
    print(f"  Particles: {N_PARTICLES}, Iterations: {N_ITERATIONS}")
    print(f"  Bounds: α={BOUNDS['alpha']}, β={BOUNDS['beta']}, γ={BOUNDS['gamma']}")

    # Initialize particles
    n_dims = 3
    lower = np.array([BOUNDS["alpha"][0], BOUNDS["beta"][0], BOUNDS["gamma"][0]])
    upper = np.array([BOUNDS["alpha"][1], BOUNDS["beta"][1], BOUNDS["gamma"][1]])

    # Random initial positions
    positions = np.random.uniform(lower, upper, (N_PARTICLES, n_dims))
    velocities = np.zeros((N_PARTICLES, n_dims))

    # Initialize personal best
    personal_best_pos = positions.copy()
    personal_best_scores = np.array([_compute_mse(p, valid_data) for p in positions])

    # Initialize global best
    global_best_idx = np.argmin(personal_best_scores)
    global_best_pos = personal_best_pos[global_best_idx].copy()
    global_best_score = personal_best_scores[global_best_idx]

    # PSO main loop
    for iteration in range(N_ITERATIONS):
        for i in range(N_PARTICLES):
            # Update velocity
            r1, r2 = np.random.random(n_dims), np.random.random(n_dims)
            cognitive = C1 * r1 * (personal_best_pos[i] - positions[i])
            social = C2 * r2 * (global_best_pos - positions[i])
            velocities[i] = W * velocities[i] + cognitive + social

            # Update position
            positions[i] = positions[i] + velocities[i]

            # Clamp to bounds
            positions[i] = np.clip(positions[i], lower, upper)

            # Evaluate fitness
            score = _compute_mse(positions[i], valid_data)

            # Update personal best
            if score < personal_best_scores[i]:
                personal_best_scores[i] = score
                personal_best_pos[i] = positions[i].copy()

                # Update global best
                if score < global_best_score:
                    global_best_score = score
                    global_best_pos = positions[i].copy()

        # Progress output every 20 iterations
        if (iteration + 1) % 20 == 0:
            print(f"  Iteration {iteration + 1}/{N_ITERATIONS}: MSE = {global_best_score:.6f}")

    # Final result
    alpha, beta, gamma = global_best_pos
    rmse = np.sqrt(global_best_score)

    print(f"\nOptimization complete!")
    print(f"  Best parameters: α={alpha:.4f}, β={beta:.4f}, γ={gamma:.4f}")
    print(f"  RMSE: {rmse:.4f} m/s")

    return WindParams(alpha=float(alpha), beta=float(beta), gamma=float(gamma))
