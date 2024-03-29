# Project Title

This is a ROS (Robot Operating System) project that uses depth and RGB images from a camera to perform certain operations.

## Files

- `depth_object_node.py`: This script subscribes to the depth and RGB images from the camera. It splits the depth image into sub-matrices, calculates the mean of each sub-matrix, and displays the RGB image with the mean values overlaid.
- `avoidance_mean_node.py`:
This script subscribes to both depth and RGB images from the camera. It then processes these images by splitting them into layers and drawing contours around objects detected in each layer. The purpose of this processing is to identify obstacles in the environment. Finally, the script implements avoidance maneuvers to navigate around any obstacles detected in its path.
  
## Dependencies

- ROS
- sensor_msgs
- rospy
- numpy
- OpenCV
- cv_bridge
- imutils

## Usage

1. Clone the repository.
2. Navigate to the repository in your terminal.
3. Run the `depth_object_node.py` script.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

[MIT](https://choosealicense.com/licenses/mit/)
