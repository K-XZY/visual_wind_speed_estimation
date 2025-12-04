"""
Parameter management for wind speed estimation.

Handles loading and saving of α, β, γ coefficients used in:
    v_wind = α * √tan(θ) + β * θ̇ + γ

This formula is derived from the force balance on a pendulum in wind:
    tan(θ) = F_drag / F_gravity = (½ρ·Cd·A·v²) / (m·g)

Solving for v: v = K · √tan(θ), where K depends on pole physical properties.
"""

import json
import os
import numpy as np
from dataclasses import dataclass, asdict

# Path to parameters file (relative to this module)
PARAMETERS_PATH = os.path.join(os.path.dirname(__file__), "parameters.json")

# Maximum angle to avoid tan(90°) = infinity (clamp to ~89°)
MAX_THETA = np.radians(89.0)


@dataclass
class WindParams:
    """Wind speed conversion parameters."""
    alpha: float = 15.0  # Coefficient for √tan(θ) term (m/s)
    beta: float = 0.1    # Angular velocity coefficient (m/s per rad/s)
    gamma: float = 0.0   # Offset (m/s)

    def calculate_wind_speed(self, theta: float, theta_dot: float) -> float:
        """
        Calculate wind speed from angle and angular velocity.

        Formula: v_wind = α * √tan(θ) + β * θ̇ + γ

        This is derived from pendulum physics where drag force balances gravity:
            tan(θ) ∝ v²  →  v ∝ √tan(θ)

        Args:
            theta: Angle from vertical (radians), clamped to [0, 89°]
            theta_dot: Angular velocity (radians/second)

        Returns:
            Estimated wind speed (m/s), minimum 0
        """
        # Clamp theta to avoid tan(90°) = infinity
        theta_clamped = np.clip(theta, 0, MAX_THETA)

        # Calculate sqrt(tan(theta))
        sqrt_tan_theta = np.sqrt(np.tan(theta_clamped))

        # Calculate wind speed
        wind_speed = self.alpha * sqrt_tan_theta + self.beta * theta_dot + self.gamma

        # Ensure non-negative wind speed
        return max(0.0, wind_speed)


def load_parameters() -> WindParams:
    """
    Load parameters from JSON file.

    Returns:
        WindParams with loaded values, or defaults if file doesn't exist
    """
    if not os.path.exists(PARAMETERS_PATH):
        return WindParams()

    try:
        with open(PARAMETERS_PATH, 'r') as f:
            data = json.load(f)
        return WindParams(
            alpha=float(data.get("alpha", 1.0)),
            beta=float(data.get("beta", 0.1)),
            gamma=float(data.get("gamma", 0.0))
        )
    except (json.JSONDecodeError, ValueError, KeyError) as e:
        print(f"Warning: Failed to load parameters: {e}")
        return WindParams()


def save_parameters(params: WindParams) -> None:
    """
    Save parameters to JSON file.

    Args:
        params: WindParams to save
    """
    os.makedirs(os.path.dirname(PARAMETERS_PATH), exist_ok=True)
    with open(PARAMETERS_PATH, 'w') as f:
        json.dump(asdict(params), f, indent=2)
    print(f"Parameters saved to {PARAMETERS_PATH}")
