# TESP 2018 Haptic Lab Project

### General overview
This is a repository for the Haptic Lab group that works with the TurtleBot3.
The main goal of this project is to provide proximity feedback to the user controlling Turtlebot3 robot.

### Prerequisites
* Desktop install of ROS is required. You can check the manual on how to install in the link below.
* Additionally, get turtlebot3 packages by `sudo apt-get install ros-kinetic-turtlebot3` and `sudo apt-get install ros-kinetic-turtlebot3-gazebo`
* To run this project you also need Python 2, to check for the version just type 
python in the command line to launch the python shell(you can exit then by pressing ctrl+d)
* To build the vib_touch packages, the fftw3 library needs to be installed beforehand: `sudo apt-get install libfftw3-dev libfftw3-doc`

### Project structure & how to run and work with ROS.

* Our project is currently placed in the tesp2018 project, for now mostly code templates are done.
* To download the project just type `git clone <name of the repository TBA>`.
* Then cd into the TESP2018 directory and type `catkin_make`, so the whole project will build. 
* After that type `source devel/setup.bash` to add ROS files included to the rospath.
* To launch real TurtleBot nodes type `roslaunch tesp2018 start.launch` (tab completion should work)
* To start the simulation you should first write `export TURTLEBOT3_MODEL=burger`, and then `roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
`
* To start the `imu_node` separately, type `rosrun tesp2018 imu_node.py`
* If you want to add additional nodes, see how `imu_node` was made (search the project)
* You should push changes to the develop branch on github.
### Useful links
* http://wiki.ros.org/
* https://www.learnpython.org/en/
* http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#turtlebot
