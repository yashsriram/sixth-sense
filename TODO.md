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
        - [x] Almost parallel to a long wall case. Just increase DISCONTINUITY THREASHOLD
        - [ ] Simple block bug

## Estimation
- [x] Estimate Sigma_N
    - [x] Give a control for some time and not errors, fit a gaussian with that
- [x] Estimate Sigma_M
    - [x] Sigma_M needs the RANSAC/LS/Corner detector

## SLAM
- [ ] SLAM
    - [x] Propogation
    - [x] Augement
    - [x] Update
    - [x] Do all updates first and then augment all new landmarks
    - [x] Periodically clean landmarks which have very less number of hits
    - [ ] Use estimated sigmas

## Planning
- [x] HitGrid
    - [x] Keep track of obstacles detected
    - [x] Is long needed for count? No.
    - [x] PRM/A* on Hitgrid
    - [x] Control
    - [x] Path smoothing

## Smooth differential agent
- [x] Handle smooth differential drive agent. Maybe just use small speeds?

## Observations
- [x] SLAM is not as transparent as first half due to matrix gymnastics
- [x] Landmarks and obstacles are not the same
    - [x] Here corners are landmarks and stored in slam state
    - [x] Lines are obstacles, endpoints stored only for planning
- [x] Use time elapsed for robot to calculate dt for propagation, augment and update
- [x] Ignore measurements while moving, do it only while stationary? Moving slowly is fine
- [x] SLAM and planning can be developed independently
- Policy
    - Initially no noise assumption, No more noise while staying stationary
    - 0 control -> doesn't add more drift
    - While true
        - Come to stop and measure
        - Find landmarks with good uncertainity (augment/update)
        - Find obstacles with good uncertainity
        - Plan a path
        - Small valued controls -> less noise, move very slowly (propogate)

## Report and demos
- [x] RK4 vs Eular stability vs dt graph
- [ ] Estimating Sigma_m does it depend on distance?
- [ ] Estimating Sigma_n does it depend on controls?
- [ ] Estimating Sigma_m while stationary only if we are taking measurements while stationary
- [ ] Check the command line compilation
- [ ] Simple rectangle
- [ ] Simple block
- [ ] Apartment
- [ ] Tune params

## Optimizations
- [ ] Use proper += -= implementations
- [ ] Merge A * B^T operation as one
- [ ] Don't create new memory for get block??

## Future work
- [ ] Circular obstacles
