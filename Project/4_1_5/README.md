# IRB-1200 Workspace Plotter

This MATLAB script computes and visualizes the positional workspace of the ABB IRB‑1200 robotic arm by sampling the first three joint angles (shoulder and elbow joints) on a configurable grid and plotting the reachable points in 3D and in orthogonal XY, XZ, and YZ projections.

---

## Features

* Configurable discretization step for joint‑angle sampling.
* Uses `forwardKinematics(params, ang_rad)` to compute end‑effector positions.
* Displays a live progress bar during computation.
* Plots:

  1. 3D isometric view of reachable points.
  2. Top (X–Y) view.
  3. Front (X–Z) view.
  4. Side (Y–Z) view.

## Requirements

* MATLAB R2019b or later
* The script `forwardKinematics.m` (implementing function `[T, origins, rots] = forwardKinematics(params, ang_rad)`).

## Script Organization

```
WorkSpace.m          % Main script
forwardKinematics.m  % FK function
```

## Usage

1. **Configure joint limits** and discretization:

   ```matlab
   % In script:
   limits = [ -170, 170;    % θ1 limits (deg)
              -100, 135;    % θ2 limits (deg)
              -200,  70 ];  % θ3 limits (deg)

   angle_step = 15;       % grid resolution in degrees
   ```

2. **Run the script**:

   ```matlab
   >> WorkSpace.m
   ```

3. **Inspect** the progress bar and wait for completion.

4. **View** the generated figure with four subplots.

## Customization

* **Resolution**: Adjust `angle_step` for finer/coarser sampling (smaller step → more points → longer runtime).
* **Sample range**: Modify `limits` to focus on specific joint subsets.
* **Wrist inclusion**: To include joints 4–6, extend loops and increase `pts` size accordingly, or switch to Monte Carlo sampling.

## Performance Tips

* Use smaller `angle_step` for a quick preview, larger for detailed maps.
* Vectorize or parallelize the loop for faster computation on large grids.
* Reduce plotted points (e.g. downsample) for faster rendering.
