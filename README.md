## README Extended Kalman Filter - P1 - Term 2

### RMSE acheived for data set #1 (see also logs/pic2.png file) 
* X: 0.0984
* Y: 0.0846
* VX: 0.3990
* VY: 0.4609

### Content

#### Files modified for orignal starter code =  https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

#### Files used to fullfil Rubic items
* 1. CarND-Extended-Kalman-Filter-Project/src/tools.cpp = tools file containing RMSE and Jacobian methods.
* 2. CarND-Extended-Kalman-Filter-Project/src/kalman.cpp = kalman class file containing filter Predict, Update and UpdateEXF methods. 
* 3. CarND-Extended-Kalman-Filter-Project/src/FusionEKF.cpp = fusion class file containing filter initialization and ProcessMeasurement. 

#### File used to analzye lidar, radar  
* CarND-Extended-Kalman-Filter-Project/src/main_file.cpp = main file reading data 'obj_pose-laser-radar-synthetic-input.txt' file to calculte RMSE for radar and lidar separately (for compilation rename it main.cpp, dont forget to save the orginal main.cpp file).
 
### Directories

* DIR CarND-Extended-Kalman-Filter-Project/src = orginal source directory including updated files from above. 
* DIR CarND-Extended-Kalman-Filter-Project/build = orginal build directory including generated executables and makefiles.
* DIR CarND-Extended-Kalman-Filter-Project/logs = logs directory including simulator screen shots and log files for radar and lidar.
* DIR CarND-Extended-Kalman-Filter-Project/data = original data directory containing the measurment file. 
