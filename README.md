<<<<<<< HEAD
#ekfslam

**Current version:** 1.0.0 

ekfslam is a real-time monocular visual-inertial SLAM system developed by YuanXingShiKong Inc, which compute the sensor's trajectory and environment sparse 3D map. 

We have tested the library in **Ubuntu 12.04** and **14.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## BLAS and LAPACK
[BLAS](http://www.netlib.org/blas) and [LAPACK](http://www.netlib.org/lapack) libraries are requiered by g2o (see below). On ubuntu:
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## Build
First, please install cyusb_linux_1.0.4 (or higher version), then go to Sensor/ directory

```
$mkdir build
$cd build
$cmake ..
$make -j8
```

go to the ekfslam/ directory
```
mkdir build
cd build
cmake ..
make -j8
```
##Usage, more details...
```
$sudo ./ekfslam configLeft.yaml
```
=======
# ovio
ovio
>>>>>>> 702a45119988fe23aa824c9765813ad706f04cc1
