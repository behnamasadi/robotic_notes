
# Camera Calibration

```
kalibr kalibr_calibrate_cameras     --bag /data/cam_april.bag --target /data/april_6x6.yaml     --models pinhole-radtan --topics /cam0/image_raw 
```


# Camera IMU Calibration

```
kalibr kalibr_calibrate_imu_camera --bag /data/cam_april.bag --cam  /data/cam_april-camchain.yaml  --imu  /data/imu.yaml  --target /data/april_6x6.yaml
```

Refs: [1](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
