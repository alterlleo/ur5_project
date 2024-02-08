# UR5-project
This is a ROS project that implements the motion for a UR5 manipulator and the data acquisition from ZedCamera 
# How to use  ```ur5_project```
In case one wishes to utilize this project, the following steps must be followed:  
- **Create a ```workspace catkin```:**
1. Open your terminal and create a new folder named ```catkin_workspace```that will become your workspace. Inside it, create the ```src``` folder;
```
mkdir catkin_workspace
```
```
cd catkin_workspace
```
```
mkdir src
```  
2. Run catkin_make;
```
mkdir src
``` 
- **Clone this repository in src:**
```
cd src
```
```
git clone https://github.com/alterlleo/ur5_project.git
```
- **Run ```ur5_generic.py```:**
```
cd <locosim path>
```
```
phyton3 ur5_generic.py
```
- **Make the object spawn:**
```
rosrun ur5_project spawning
```
- **Run vision:**
1. Open a new terminal and run:  
```
rosrun ur5_project vision.py
```
- **Run ```project``` :**  
```
rosrun ur5_project project
```
