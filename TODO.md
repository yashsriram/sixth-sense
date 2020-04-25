# Plan
- [x] Smooth the workflow
    - [x] Find a good linear algebra library
    - [x] Write operator overloads, extensions
    - [x] Test EJML and extensions
    - [x] Translate all src to Kotlin
    - [x] Remove Mat2 Vec2 Vec3
    - [x] Find command line compile and run commands
- [x] Reproduce HW2 EKF SLAM in java using processing.
- [ ] Map corner features from lines
    - [ ] implement RANSAC from class
        - [ ] Change do-while to while
    - [ ] The least squares line fitting
    - [ ] Case based corner detection (using discontinuities, intersections etc...)
        - [ ] Break obstacles using discontinuties
        - [ ] Cull ghost intersection landmarks
        - [ ] What is 501?
        - [ ] for (i in 0 until lineSegments.size) {?
        - [ ] breaking cases
    - [ ] Optimize this step
- [ ] Estimate Sigma_N and Sigma_M from simulator
    - [ ] Give a control for some time and not errors, fit a gaussian with that
    - [ ] Sigma_M needs the RANSAC/LS/Corner detector
- [ ] Landmarks and obstacles are not the same
- [ ] SLAM vs Planning
    - [ ] Here corners are landmarks and stored as slam state
    - [ ] Lines are obstacles, endpoints stored for only planning
    - [ ] RRT for some time. Start going to furthest node until a new obstacle is detected?
    - [ ] Planning not the top priority rn
- [ ] Circular obstacles
- [ ] Dynamic human obstacles

# Efficiency
- [ ] Use proper += -= implementations
- [ ] Merge A * B^T operation as one
- [ ] Don't create new memory for get block??

# Simulator
- [ ] significance of time in odometry and laser scan struct
- [x] scaling
- [x] 3d rendering

