# UR5-project
This is a ROS project that implements the motion for a UR5 manipulator and the data acquisition from ZedCamera.  
It is written for Ubuntu 20 and relies on the following setup: https://github.com/mfocchi/lab-docker
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
catkin_make
``` 
- **Clone this repository in src:**
```
cd src
```
```
git clone https://github.com/alterlleo/ur5_project.git
```
- **Ensure that the environment is properly configured for the use of ROS packages within your workspace:**  
This can be done by running the following instruction in the catkin_workspace directory:
```
source devel/setup.bash
```
- **Run ```ur5_generic.py```:**
```
cd <locosim path>
```
```
phyton3 ur5_generic.py
```
- **Make the object spawn:**
1. Open a new terminal and run:  
```
source <add path of catkin_workspace/>devel/setup.bash
```
```
rosrun ur5_project spawning
```
- **Run vision:**
1. Open a new terminal and run:
```
source <add path of catkin_workspace/>devel/setup.bash
``` 
```
rosrun ur5_project vision.py
```
- **Run ```project``` :**
1. Now you can run ```project``` :
```
rosrun ur5_project project
```
Please note: It may not work on the first compilation attempt. Don't worry and try again!
