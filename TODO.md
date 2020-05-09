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

## Simulator
- [x] significance of time in odometry and laser scan struct
- [x] scaling
- [x] 3d rendering

## Obstacle and landmark extraction
- [x] Extract obstacles
    - [x] RANSAC/LS
        - [x] Return end points not defining points
        - [x] Line fitting
        - [x] Fix partition code
        - [x] use IEP for better partitioning
    - [x] IEP
- [x] Extract landmarks
    - [x] No need for checking intersections separately due to IEP
    - [x] Loose end detection in first stage
    - [x] Intersection detection in second stage
- [ ] Advanced landmark detection
    - [ ] Intersection landmarks
        - [ ] use perpendicular projections points
        - [x] check distances (IEP shortcoming)
    - [ ] Loose endpoints
        - [ ] Sense line check (almost parallel to wall case)

## Estimation
- [ ] Estimate Sigma_N and Sigma_M from simulator
    - [ ] Give a control for some time and not errors, fit a gaussian with that
    - [ ] Sigma_M needs the RANSAC/LS/Corner detector

## SLAM
- [ ] SLAM
    - [x] Propogation
    - [x] Augement
    - [x] Update
    - [ ] Use better augment update like bobby's
    - [x] Periodically clean landmarks which have very less number of hits
    - [ ] Tune params

## Planning
- [ ] Keep track of obstacles detected
- [ ] HitGrid
    - [ ] Octa map
    - [x] Is long needed for count? No.
    - [ ] Use multiple measrurements over time to form a robot.sensing hitmap from which extract landmarks
    - [ ] Use two hitgrids, one for planning one for robot.sensing
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
- [ ] Planning
    - [ ] RRT for some time. Start going to furthest node until a new obstacle is detected?
    - [ ] Planning not the top priority rn

## Smooth differential agent
- [ ] Handle smooth differential drive agent. Maybe just use small speeds?

## Observations
- [x] SLAM is not as transparent as first half due to matrix gymnastics
- [x] Landmarks and obstacles are not the same
    - [x] Here corners are landmarks and stored in slam state
    - [x] Lines are obstacles, endpoints stored only for planning
- [x] Use time elapsed for robot to calculate dt for propagation, augment and update
- [x] RK4 vs Eular stability vs dt graph
- [x] Ignore measurements while moving, do it only while stationary? Moving slowly is fine
- [x] SLAM and planning can be developed independently

## Considerations
- [ ] Estimating Sigma_m does it depend on distance?
- [ ] Estimating Sigma_n does it depend on controls?
- [ ] Estimating Sigma_m while stationary only if we are taking measurements while stationary

## Demos
- [ ] Check the command line compilation
- [ ] Simple rectangle
- [ ] Simple block
- [ ] Apartment

## Optimizations
- [ ] Use proper += -= implementations
- [ ] Merge A * B^T operation as one
- [ ] Don't create new memory for get block??

## Future work
- [ ] Circular obstacles
