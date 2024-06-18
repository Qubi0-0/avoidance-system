# UAV Avodiance system based on Potential Fields Method

This is a ROS (Robot Operating System) project that uses stereovision camera for collision avoidance.

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

# Rviz Visualization 

![Path Visualization]{Images/pointcloudgazeborviz-after.png}

# Path Visualization in QgroundControl

![Path Visualization]{Images/pfa-qg-path.png}

# Path Visualization in Rviz

![Rviz Visualization of working system]{Images/pfa-rviz-path.png}

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

[MIT](https://choosealicense.com/licenses/mit/)
