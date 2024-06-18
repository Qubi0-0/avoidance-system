# UAV Avodiance system based on Potential Fields Method

This is a ROS (Robot Operating System) project that uses stereovision camera for collision avoidance. The project was developed for my thesis project.

## Files

# Cpp Avoidance

- `avoidance.cpp`: Main part of avoidance node. It computes all nessesary data to change UAV trajectory. It uses Potential Fields Method as a core for collision avoidance system.   

- `mothernode.py`: Subscribes to avoidance messages to control drone path. It was seperated from `avoidance.cpp` due to testing with other systems.  

- `octomap.cpp`: Allows to compute new waypoint for UAV. It takes octomap to find a new sub-waypoint.  

- `path.cpp`: Subscribes to drone position and convert it to path. Used for Path visualization in Rviz.

  
## System In Action

# Performance test in SITL

Message potienital\_twist represents result of calculations for PFM. It shows how frequent the system is able to respond.

| **Message**         | **Avg. value [Hz]** | **Min [s]** | **Max [s]** | **Std. Deviation [s]** |
|---------------------|---------------------|-------------|-------------|------------------------|
| potential\_twist    | 3.410               | 0.143       | 0.474       | 0.06600                |

To represent how it works, there are few images obtained after testing. 

# Rviz Visualization of a working system

![Rviz Visualization ](https://github.com/Qubi0-0/avoidance-system/blob/main/Images/pointcloudgazeborviz-after.png)

# Path Visualization in QgroundControl

![!\[Path Visualization\]{Images/pfa-qg-path.png}](https://github.com/Qubi0-0/avoidance-system/blob/main/Images/pfa-qg-path.png)

# Path Visualization in Rviz

![!\[Rviz Visualization of working system\]{Images/pfa-rviz-path.png}](https://github.com/Qubi0-0/avoidance-system/blob/main/Images/pfa-rviz-path.png)

Octomap is used only for better visualization of trajectory.

## How to use

No special Roslaunches are needed to make it work. Make sure to takeoff independently from avoidance system. 

# Before start 
make sure that subscribed topic from drone is correctly set. It might differ from avaiable version here.

# Running system

1. All ususal steps for ros packages such as 

```bash
source /opt/ros/noetic/setup.bash
``` 
Are still needed.

2. Start `avoidance_node`, responsible for computing results from depth camera.

```bash
rosrun cpp_avoidance avoidance_node  
```

3. and in new terminal start `mother_node`, responisble for decision-making.

```bash
rosrun cpp_avoidance mother_node
```


## License

[MIT](https://choosealicense.com/licenses/mit/)
