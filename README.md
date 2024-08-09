## Requirements
- ROS Noetic Ninjemys 
- Gazebo
- Python 3
- Cython (need for Tess)
- Tess (Python) (https://github.com/wackywendell/tess)
- Hector Quadrotor (Optional) (https://github.com/RAFALAMAO/hector-quadrotor-noetic.git)

## Explanation
# 3D CVT
 Match the agents used in ```8_drone_cbf_cvt.py``` to name setup on Gazebo. Obstacles' positions are set in this file so please edit them when using it. The CVT and CBF properties are defined in ```voronoiAndBarrier.py```, and ```voronoiAndBarrierV2.py``` are prototype for super-ellipsoidal CBF. Plotting and graphing scripts are in 
```plot_scripts```. 

1. Put ```scripts``` and ```launch```folder into your ROS project folder inside the workspace folder.
2. Put ```hector_gazebo_worlds``` in your world folder.
3. Launch the robots and Gazebo
```sh
roslaunch [your folder] quadrotor_crate_symmetry.launch
```
4. Run the script
```sh
rosrun [your folder] 8_drone_cbf_cvt.py
```
or run it directly after the gazebo is launch.

# Spherical CVT
 The code are only in python scripts. Everything is defined in ```sphereV.py```. ```.csv``` files will be created, so use ```plot_[xxx].py``` files to plot results and ```gif.py``` to combined created images as ```.gif```.

## Prototypes
Python scripts in ```python-test-project```, just run them directly. There are parameterized version of CVT with density function and super-ellipsoid CBF.
