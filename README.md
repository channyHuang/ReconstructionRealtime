Realtime Reconstructon Base on R3LIVE In Windows without ROS environment.  

Using AirSim to generate and get scene data, include IMU、Lidar、RGB image data, instead of ROS data.

# Dependency
* **OpenSceneGraph** (show)
* **GLFW3** (imgui as ui)
* **OpenCV** (read image from scene)
* **PCL** (R3LIVE required)
* **Boost** (R3LIVE required)
* **Eigen3** (R3LIVE required)
* **ffmpeg** (image processing)
```
set(ffmpeg_DIR "E:/thirdLibs/installers/ffmpeg-6.0" CACHE PATH "")
```
ffmpeg_DIR to set path of `ffmpeg` folder
* **CGAL**
```
set(CGAL_DIR "E:/thirdLibs/CGAL-5.4-beta1/include" CACHE PATH "")
``` 
CGAL_DIR to set path of `CGAL` header only library
* **AirSim**
```
set(AirSim_DIR "E:/thirdLibs/AirSim")
```
AirSim_DIR to set path of `AirSim` folder


# Outer Dependency
* commonClass (in my other rep)
In CMakeLists.txt, 
```
set(OUTER_DIR ${CMAKE_SOURCE_DIR}/../commonClass)
```
use to set path of `commonClass` folder

# Notes
## Optical_flow
Image feature tracker using OpenCV. Lidar points project to image to get m_current_tracked_pts in current image frame, than calculate fundamental matrix and frame pose.

## loam
IMU processing. **But** IMU data in AirSim seems to be error or drift obviously...

## meshing
Mesh Reconstruction is similar to openMVS. 