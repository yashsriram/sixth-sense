# Plan
- [ ] Find a good linear algebra library
    - [x] Write operator overloads, extensions
    - [x] Test EJML and extensions
    - [ ] Translate all src to Kotlin
    - [ ] Find command line compile and run commands
    - [ ] Kotlinize everything
    - [ ] Remove Mat2 Vec2 Vec3
- [ ] Reproduce HW2 EKF SLAM in java using processing.
- [ ] How to build line features from point features? Maybe detect end points. Maybe use RANSAC for that?
- [ ] How to navigate? RRT for some time. Start going to furthest node until a new obstacle is detected?
- [ ] Dynamic human obstacles
- [ ] Circular obstacles

# Efficiency
- [ ] Use proper += -= implementations
- [ ] Merge A * B^T operation as one
- [ ] Don't create new memory for get block??

# Simulator
- [ ] significance of time in odometry and laser scan struct
- [x] scaling
- [x] 3d rendering

