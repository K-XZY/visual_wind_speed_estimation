# Pole Angle Wind Speed Estimation

## Goal

Estimate wind speed by measuring the deflection angle of a rigid pole that hangs as a pendulum, pivoting from the top. The stronger the wind, the larger the angle of deflection.

## Physical Setup

```
    Pivot Point (fixed)
         │
         │  ← Pole (rigid pendulum)
         │
        ╱│╲
       ╱ │ ╲
      ╱  │  ╲  ← Wind pushes pole
     ╱ θ │   ╲
    ╱    │    ╲
   ●─────┼─────●
         │
         ▼
      Vertical
      (θ = 0)
```

- **Pole**: Rigid pendulum hanging from a fixed pivot point at the top
- **Two green markers**: Tape wrapped around the pole at two positions along its length
- **Camera**: Raspberry Pi camera (with IMX477 sensor) observing the pole
- **Angle reference**:
  - θ = 0 → pole hangs vertically (no wind)
  - θ = π/2 → pole is horizontal (extreme wind)

## Detection Method

### 1. Color-Based Marker Detection

The system uses HSV (Hue, Saturation, Value) color filtering to isolate the green markers:

- Convert camera frame from RGB to HSV color space
- Apply threshold to extract green pixels:
  - Lower bound: `[35, 50, 50]` (H, S, V)
  - Upper bound: `[85, 255, 255]` (H, S, V)
- Clean the binary mask using morphological operations (close/open)

### 2. Centroid Calculation

For each detected green region:

- Find contours in the binary mask
- Filter by minimum area to reject noise
- Calculate centroid using image moments:

$$
c_x = \frac{M_{10}}{M_{00}}, \quad c_y = \frac{M_{01}}{M_{00}}
$$

where $M_{ij}$ are the image moments of the contour.

### 3. Angle Calculation

Given two marker centroids $P_1 = (x_1, y_1)$ and $P_2 = (x_2, y_2)$:

$$
\Delta x = x_2 - x_1, \quad \Delta y = y_2 - y_1
$$

The angle from horizontal:

$$
\phi = \arctan2(\Delta y, \Delta x)
$$

The angle from vertical:

$$
\theta = \left| \frac{\pi}{2} - \phi \right|
$$

## Wind Speed Estimation

### Conversion Formula

Wind speed is estimated using a linear combination of the angle and its rate of change:

$$
v_{wind} = \alpha \cdot \theta + \beta \cdot \dot{\theta} + \gamma
$$

Where:
- $v_{wind}$ — estimated wind speed (m/s)
- $\theta$ — angle from vertical (radians)
- $\dot{\theta}$ — angular velocity (radians/second)
- $\alpha$ — coefficient for angle contribution (m/s per radian)
- $\beta$ — coefficient for angular velocity contribution (m/s per rad/s)
- $\gamma$ — offset/bias term (m/s)

### Coefficient Interpretation

| Coefficient | Physical Meaning | Units |
|-------------|------------------|-------|
| α | Steady-state response: how much wind speed per unit angle | m·s⁻¹·rad⁻¹ |
| β | Dynamic response: accounts for pole inertia and damping | m·s⁻¹·(rad/s)⁻¹ |
| γ | Offset: corrects for bias (e.g., pole not perfectly balanced) | m/s |

### Why Include θ̇?

The angular velocity term $\dot{\theta}$ helps account for:

1. **Transient response**: Wind gusts cause the pole to accelerate before reaching equilibrium
2. **Damping effects**: A moving pole experiences different aerodynamic forces than a stationary one
3. **Improved accuracy**: During changing wind conditions, θ alone lags behind the true wind speed

## Configuration Parameters

```python
# Wind speed conversion coefficients
alpha = 1.0    # Angle coefficient (m/s per radian)
beta = 0.1     # Angular velocity coefficient (m/s per rad/s)
gamma = 0.0    # Offset (m/s)

# Detection parameters
lower_green = [35, 50, 50]    # HSV lower bound
upper_green = [85, 255, 255]  # HSV upper bound
min_area = 100                # Minimum contour area (pixels)

# Camera settings
resolution = (1280, 720)
fps = 30
```

## Output

- **Real-time display**: Video feed with detected markers, angle overlay, and live plot
- **Video recording**: MP4 file with combined video and angle graph
- **Data export**: CSV file with timestamped angle measurements

## Future Work

- Calibrate α, β, γ coefficients against a reference anemometer
- Investigate nonlinear models if linear approximation proves insufficient
- Add wind direction estimation (requires additional sensing or assumptions)
