# turtle_catch

## Important information
ROS package expanding turtlesim options - allows to run turtlesim with a turtle operated by arrow keys and an autonomous turtle chasing ours. Distance is plotted with rqt\_plot.

### Prerequisites
To start with this project, ROS noetic needs to be installed and sourced on the computer.

### Running the program
After cloning this repository to your computer (and to your workspace's 'src' directory), simply run the command in a terminal:
```
roslaunch turtle_catch turtle_catch.launch
```
You might need to source the workspace:
```
source devel/setup.bash
```

### Additional scripts
An additional script (process\_chase.py) has been added to process a bagfile with recorded topics of a chase. The script can be run separately:
```
./process_chase.py input_bagfile.bag 
```
The output will be a processed bagfile (processed\_chase.bag) with remapped topics (/runner/pose and /chaser/pose) and a set of information regarding covered distance, duration of a chase and average velocity of each turtle.

### Authors
* **Emilia Szymańska**
