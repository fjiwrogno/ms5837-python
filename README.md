# MS5837 depth sensor drive and ros node 
- topic name: depth_sensor_node/depth
- frequency： 100hz(default, greatly depends on the sampling rate of read)  
## Content
acccept the depth sensor from ms5837 30ba sensor that is connected with vim4 through I2C.
then deploy a low-pass filter to get smoother value.
## usage pipeline
```
rosrun ms5837_ros ms5837_node.py
```
Then, calibrate the depth sensor when on land or just above the water surface or other specific water level for other purposes.
## todo 
- [] parameter reconfig through ros param
- [] field test for identifying the frequency of each sampling rate