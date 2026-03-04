# ShotCalculator Equations Visualization

This document visualizes the mathematical equations used in `ShotCalculator.java` for shoot-on-the-move calculations.

---

## 1. Robot Field Velocity (Translation)

Convert robot-relative velocity to field-relative coordinates:

$$
\vec{v}_{\text{robot, field}} = \begin{bmatrix} v_x \\ v_y \end{bmatrix}_{\text{robot}} \cdot R(\theta_{\text{robot}})
$$

Where:
- $v_x, v_y$ = robot-relative velocities (m/s)
- $\theta_{\text{robot}}$ = robot heading (radians)
- $R(\theta)$ = rotation matrix

---

## 2. Shooter Velocity (Field Frame)

Calculate the shooter's velocity including rotational contribution:

$$
v_{\text{shooter, x}} = v_{\text{robot, field, x}} + \omega \cdot (d_y \cos(\theta) - d_x \sin(\theta))
$$

$$
v_{\text{shooter, y}} = v_{\text{robot, field, y}} + \omega \cdot (d_x \cos(\theta) - d_y \sin(\theta))
$$

Where:
- $\omega$ = angular velocity (rad/s)
- $d_x, d_y$ = shooter offset from robot center (m)
- $\theta$ = robot heading (rad)

**Physical meaning:** The shooter inherits the robot's translational velocity plus additional velocity from rotation about the robot center.

---

## 3. Initial Distance to Target

$$
D_{\text{initial}} = \sqrt{(x_{\text{target}} - x_{\text{shooter}})^2 + (y_{\text{target}} - y_{\text{shooter}})^2}
$$

Where:
- $(x_{\text{target}}, y_{\text{target}})$ = target position
- $(x_{\text{shooter}}, y_{\text{shooter}})$ = current shooter position

---

## 4. Time of Flight Lookup

$$
t_{\text{flight}} = \text{interpolate}(D_{\text{initial}}, \text{timeOfFlightMap})
$$

This is a **lookup table interpolation** based on empirically measured or simulated flight times at various distances.

---

## 5. Lookahead Position (Velocity Compensation)

Project where the shooter will be when the note arrives at the target:

$$
\vec{P}_{\text{lookahead}} = \vec{P}_{\text{shooter}} + \vec{v}_{\text{shooter}} \cdot t_{\text{flight}}
$$

Expanded:

$$
x_{\text{lookahead}} = x_{\text{shooter}} + v_{\text{shooter, x}} \cdot t_{\text{flight}}
$$

$$
y_{\text{lookahead}} = y_{\text{shooter}} + v_{\text{shooter, y}} \cdot t_{\text{flight}}
$$

**Physical meaning:** Because the robot imparts its velocity to the launched note, we need to aim from where the shooter *will be* when the note reaches the target, not where it is now.

---

## 6. Lookahead Distance

$$
D_{\text{lookahead}} = \sqrt{(x_{\text{target}} - x_{\text{lookahead}})^2 + (y_{\text{target}} - y_{\text{lookahead}})^2}
$$

---

## 7. Effective Distance (Clamped)

$$
D_{\text{effective}} = \text{clamp}(D_{\text{lookahead}}, D_{\text{min}}, D_{\text{max}})
$$

$$
D_{\text{effective}} = \max(D_{\text{min}}, \min(D_{\text{max}}, D_{\text{lookahead}}))
$$

This prevents extrapolation beyond the tested range of the interpolation maps.

---

## 8. Target Heading

Calculate the angle from lookahead position to target, then rotate 180° (shooter is at back of robot):

$$
\theta_{\text{target}} = \text{atan2}(y_{\text{target}} - y_{\text{lookahead}}, x_{\text{target}} - x_{\text{lookahead}}) + \pi
$$

---

## 9. Mechanism Setpoints (Interpolation)

$$
\text{hoodAngle} = \text{interpolate}(D_{\text{effective}}, \text{shotHoodAngleMap})
$$

$$
\text{flywheelSpeed} = \text{interpolate}(D_{\text{effective}}, \text{shotFlywheelSpeedMap})
$$

$$
t_{\text{flight, final}} = \text{interpolate}(D_{\text{effective}}, \text{timeOfFlightMap})
$$

---

## 10. Validity Check

$$
\text{isValid} = (D_{\text{lookahead}} \geq D_{\text{min}}) \land (D_{\text{lookahead}} \leq D_{\text{max}})
$$

---

## Complete Data Flow Diagram

```
Robot Pose (x, y, θ) ──┐
                       ├──> Shooter Position (x_s, y_s)
Shooter Offset (d_x, d_y) ──┘

Robot Velocity (v_x, v_y, ω) ──> Field Velocity ──> Shooter Velocity (v_sx, v_sy)
                                       │
                                       └──> Lookahead Position = Shooter Position + Velocity × TOF
                                                      │
Target Position (x_t, y_t) ──────────────────────────┼──> Distance (lookahead)
                                                      │
                                                      ├──> Hood Angle (interpolate)
                                                      ├──> Flywheel Speed (interpolate)
                                                      ├──> Target Heading (atan2 + 180°)
                                                      └──> Time of Flight (interpolate)
```

---

## Coordinate Frames

- **Robot Frame:** x-forward, y-left, origin at robot center
- **Field Frame:** x-downfield, y-left (from driver station perspective), origin at field corner
- **Rotation Direction:** Counter-clockwise positive (standard mathematical convention)

---

## Units

| Variable | Unit |
|----------|------|
| Position (x, y) | meters (m) |
| Velocity (v) | meters per second (m/s) |
| Angular velocity (ω) | radians per second (rad/s) |
| Angle (θ) | radians (rad) |
| Time of flight | seconds (s) |
| Hood angle | degrees (°) |
| Flywheel speed | RPM |
| Distance | meters (m) |
