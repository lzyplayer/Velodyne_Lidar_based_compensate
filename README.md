### LiDAR_self_compensated

------

compensate LiDAR data mainly on yaw Rotation  

1. set bagfile path in`lidar_compensated.m`
2. set lidar topic in `read_fullpc_from_bag.m`
3. run `lidar_compensated.m` to start compensating.
4. lidar odometry calculated by ICP