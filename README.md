# Autoware Toy Project
blog : https://blog.naver.com/dangmu1996
---
## Control with joystick
+ start the AWSIM

+ I am using logitech joystick
+ What to execute
```
$ ros2 run joy joy_node
$ ros2 run joystick_control joystick_interface
```

---
## 3D SLAM
+ Start the AWSIM

```
// This will start joy driver node. joystick interface, loading URDF, converting imu data,START SLAM, all at once
$ ros2 launch dangmu slam_launch.py

// to save the map
$ ros2 service call /map_save std_srvs/Empty
```

+ 도저히 map 전체다는 못찍겠더라
![스크린샷 2023-12-25 20-15-34](https://github.com/rsasaki0109/lidarslam_ros2/assets/79675698/a2e4f747-0b55-4c65-8f57-44ca2abaf3cd)