+++
+++

<figure>
<img src="landmark-refinement-using-iep.gif" alt="Landmark refinement using iterative endpoint method">
<figcaption>Landmark refinement using iterative endpoint method</figcaption>
</figure>

For a differential drive robot with noisy lidar and control, we use Extended Kalman Filter to perform simultaneous localization and mapping (SLAM).
Notably we use [Douglas–Peucker algorithm](https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm) to refine landmarks used for SLAM state.
