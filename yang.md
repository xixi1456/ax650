# 自瞄改动

## 标定：通过capture_calibration_Data.py文件进行采集图像和imu数据，然后通过matlab进行标定(intrinsics = cameraParams.Intrinsics;[K,dist] = cameraIntrinsicsToOpenCV(intrinsics);)，得到相机k和dist，然后再把./mat文件让python3 matlab_calib_to_yml.py --mat matlab.mat -o cam_param_new.yml进行转换
 
