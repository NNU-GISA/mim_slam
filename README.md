### Code style is similar to LOAM, including three steps:
#### stablePointExtract
    extract stable point using LeGO-LOAM method
#### laserOdometry
    estimate the pose between two consecutive frames using point-to-plane ICP
#### laserMapping
    optimize above pose estimation using frame-to-submap matching, and considering loop closure
    
