# online_photometric_calib

This is just a ROS version of [online_photometric_calibration](https://github.com/tum-vision/online_photometric_calibration). We make it for easy test with rosbag environment.

If you think it's helpful, please cite their paper:  

Online Photometric Calibration of Auto Exposure Video for Realtime Visual Odometry and SLAM (P. Bergmann, R. Wang, D. Cremers), In IEEE Robotics and Automation Letters (RA-L), volume 3, 2018.

----
Modifications: 
- this package not include the offline optimization
- this package not read ground truth time
- check **/tracking_img** topic for tracking feature history
- check **/correct_img** topic for corrected image, for future feature tracking

Notes:
- existing issues: [NonLinearOptimizer crash](https://github.com/tum-vision/online_photometric_calibration/issues/7), sometimes it happen in euroc v1_03_diffcult. maybe ok in another sequences.
- you can comment out **Line 221** at opc_node.cpp to disable the optimization, only see their robust KLT tracking if you want to try feature tracking in illumination changing environment.