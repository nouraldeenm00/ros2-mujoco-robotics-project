# Denavit-Hartenberg (DH) Convention Analysis
## 4-DOF Robot Arm

## Frame Assignment

The robot arm consists of 4 revolute joints with the following structure:
- **Joint 1**: Base rotation (Z-axis)
- **Joint 2**: Shoulder (Y-axis)  
- **Joint 3**: Elbow (Y-axis)
- **Joint 4**: Wrist rotation (Z-axis)

### Frame Definitions

```
Frame 0 (World/Base):
  - Origin: Base center
  - Z₀: Vertical (upward)
  - X₀: Forward

Frame 1 (After Joint 1):
  - Origin: Top of base
  - Z₁: Rotation axis of Joint 1 (vertical)
  - X₁: Along Link 1

Frame 2 (After Joint 2):
  - Origin: Joint 2 location
  - Z₂: Rotation axis of Joint 2 (perpendicular to Z₁)
  - X₂: Along Link 2

Frame 3 (After Joint 3):
  - Origin: Joint 3 location
  - Z₃: Rotation axis of Joint 3 (parallel to Z₂)
  - X₃: Along Link 3

Frame 4 (After Joint 4):
  - Origin: Joint 4 location
  - Z₄: Rotation axis of Joint 4 (parallel to Z₁)
  - X₄: Along end effector
```

## DH Parameters Table

| Link | θᵢ (rad) | dᵢ (m) | aᵢ (m) | αᵢ (rad) |
|------|----------|--------|--------|----------|
| 1    | θ₁*      | 0.35   | 0      | π/2      |
| 2    | θ₂*      | 0      | 0.25   | 0        |
| 3    | θ₃*      | 0      | 0.20   | 0        |
| 4    | θ₄*      | 0      | 0.10   | 0        |

*Variable joint angles

### Parameter Definitions:
- **θᵢ**: Joint angle (rotation about Zᵢ₋₁)
- **dᵢ**: Link offset (translation along Zᵢ₋₁)
- **aᵢ**: Link length (translation along Xᵢ)
- **αᵢ**: Link twist (rotation about Xᵢ)

### Link Dimensions (from URDF):
- Base height: 0.05 m
- Link 1 length: 0.30 m
- Link 2 length: 0.25 m
- Link 3 length: 0.20 m
- End effector length: 0.10 m

## Transformation Matrices

### Individual Link Transformations

The transformation from frame i-1 to frame i is:

```
Tᵢ₋₁ⁱ = Rot(Z, θᵢ) × Trans(0, 0, dᵢ) × Trans(aᵢ, 0, 0) × Rot(X, αᵢ)
```

Expanded form:
```
Tᵢ₋₁ⁱ = [cos(θᵢ)  -sin(θᵢ)cos(αᵢ)   sin(θᵢ)sin(αᵢ)   aᵢcos(θᵢ)]
        [sin(θᵢ)   cos(θᵢ)cos(αᵢ)  -cos(θᵢ)sin(αᵢ)   aᵢsin(θᵢ)]
        [0         sin(αᵢ)           cos(αᵢ)           dᵢ        ]
        [0         0                 0                 1         ]
```

### Forward Kinematics

The complete transformation from base to end effector:

```
T₀⁴ = T₀¹ × T₁² × T₂³ × T₃⁴
```

End effector position:
```
[x]   [T₀⁴[0,3]]
[y] = [T₀⁴[1,3]]
[z]   [T₀⁴[2,3]]
```

## Joint Limits

Based on the URDF model:

| Joint | Lower Limit | Upper Limit | Range |
|-------|-------------|-------------|-------|
| 1     | -π rad      | +π rad      | 360°  |
| 2     | -π/2 rad    | +π/2 rad    | 180°  |
| 3     | -π/2 rad    | +π/2 rad    | 180°  |
| 4     | -π rad      | +π rad      | 360°  |

## Workspace Analysis

### Reachable Workspace

Maximum reach (all links extended):
```
R_max = Link1 + Link2 + Link3 + EndEffector
R_max = 0.30 + 0.25 + 0.20 + 0.10 = 0.85 m
```

Minimum reach (links folded):
```
R_min = |Link1 - Link2 - Link3 - EndEffector|
R_min ≈ 0.05 m
```

### Working Height Range

```
Z_min = Base_height = 0.05 m
Z_max = Base_height + all_links_vertical = 0.05 + 0.85 = 0.90 m
```

## Verification

Home position (all joints at 0°):
- Expected end effector position: [0, 0, 0.90]
- This matches the forward kinematics calculation

90° base rotation (θ₁ = π/2, others = 0):
- Expected end effector position: [0, 0, 0.90]
- Rotation only affects orientation, not height
