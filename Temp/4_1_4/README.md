# IRB-1200 Joint-Pose Visualizer

This MATLAB script visualizes a series of predefined joint configurations for the ABB IRB‑1200 robotic arm. For each pose, it spawns a new figure showing the links and coordinate frames of every joint.

---

## Features

* **Predefined Poses**: Displays seven example configurations of θ₁…θ₆ (in degrees).
* **Forward Kinematics**: Calls `forwardKinematics(params, ang_rad)` to compute link origins and rotations.
* **Joint Frames**: Renders X, Y, Z axes at each joint with configurable arrow length.
* **Per-Pose Figures**: Opens a separate window for each test angle set, titled by its joint values.

## Requirements

* MATLAB R2019b or later
* `forwardKinematics.m` implementing:

  ```matlab
  [T, origins, rots] = forwardKinematics(params, ang_rad)
  ```

## Script Files

```
plot5points.m            % Main visualization script
forwardKinematics.m      % Robot FK function
```

## Usage

1. **Configure DH constants** at the top of `plot5points.m`:

   ```matlab
   params = struct('d1',39.9,'a2',35,'a3',4.2,'d4',35.1,'d6',8.2);
   ```

2. **Define test poses** as a matrix of size N×6 (degrees):

   ```matlab
   test_angles = [ ...
        0   0    0   0   0   0;
       90   0    0   0   0   0;
       ...
   ];
   ```

3. **Adjust arrow length** and style if desired:

   ```matlab
   axisLen = 5;                   % arrow length in cm
   quivOpts = {'AutoScale','off', 'MaxHeadSize',1,'LineWidth',1.2};
   ```

4. **Run** the script:

   ```matlab
   >> plot5points
   ```

Each pose will appear in its own figure labeled “Pose \[θ₁ θ₂ θ₃ θ₄ θ₅ θ₆]°.”

## Customization

* **Add or remove poses** by editing `test_angles`.
* **Change view angle** in the `view(45,30)` call.
* **Color or arrow settings**: Modify `cols3` and `quivOpts` for different axis visuals.
