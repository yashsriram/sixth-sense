# Plan

## Foundations
- [x] Smooth the workflow
    - [x] Find a good linear algebra library
    - [x] Write operator overloads, extensions
    - [x] Test EJML and extensions
    - [x] Translate all src to Kotlin
    - [x] Remove Mat2 Vec2 Vec3
    - [x] Find command line compile and run commands
- [x] Reproduce HW2 EKF SLAM in java using processing.
- [x] Replace DMatrix2 with FMatrix2
- [x] Landmarks and obstacles are not the same
    - [x] Here corners are landmarks and stored in slam state
    - [x] Lines are obstacles, endpoints stored only for planning
- [x] SLAM is not as transparent as first half due to matrix gymnastics

## Simulator
- [ ] significance of time in odometry and laser scan struct
- [x] scaling
- [x] 3d rendering

## Obstacle and landmark extraction
- [ ] Extract obstacles
    - [x] RANSAC/Linefitting
        - [x] Change do-while to while
        - [x] Partition based on discontinuity
        - [x] Return end point not defining points
        - [ ] Line fitting
    - [ ] IEP
    - [ ] HitGrid
        - [ ] Octa map
        - [x] Is long needed for count? No.
        - [ ] Use multiple measrurements over time to form a sensing hitmap from which extract landmarks
        - [ ] Use two hitgrids, one for planning one for sensing
        - Policy
            - Initially no noise assumption, No more noise while staying stationary
            - 0 control -> doesn't add more drift
            - While true
                - Come to stop and measure
                - Find landmarks with good uncertainity (augment/update)
                - Find obstacles with good uncertainity
                - Plan a path
                - Small valued controls -> less noise, move very slowly (propogate)
        - [ ] Revert v, w, vdot_max, wdot_max to good values before proceeding
- [ ] Extract landmarks
    - [ ] Case based landmark detection (using discontinuities and intersections)
        - [ ] Fix line segment line segment intersection
        - [ ] Fix spurious detections

## Considerations
- [ ] Estimating Sigma_m while stationary only if we are taking measurements while stationary
- [ ] Estimating Sigma_n does it depend on controls? Should not be ideally
- [ ] Ignore measurements while moving? maybe. Try doing it only while stationary
- [ ] Move slowly and take frequent stops for landmark measurements
- [ ] SLAM and planning can be developed independently

## Estimation
- [ ] Estimate Sigma_N and Sigma_M from simulator
    - [ ] Give a control for some time and not errors, fit a gaussian with that
    - [ ] Sigma_M needs the RANSAC/LS/Corner detector

## SLAM
- [ ] SLAM

## Planning
- [ ] Planning
    - [ ] RRT for some time. Start going to furthest node until a new obstacle is detected?
    - [ ] Planning not the top priority rn

## Advanced
- [ ] Circular obstacles
- [ ] Dynamic human obstacles

## Optimizations
- [ ] Use proper += -= implementations
- [ ] Merge A * B^T operation as one
- [ ] Don't create new memory for get block??
