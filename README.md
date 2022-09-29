# Camera-Calibration-and-RBGD-Point-cloud-generation-from-Stereo-Camera-Input
Camera Calibration of my phone camera, Disparity image and RBG-D point cloud generation and visualization by consuming Stereo camera data stored in a ROSBAG file

PS - The stereo input ROSBAG file is not included in the uploaded file since the file size was too large. 

# Steps to run the file 

Requirement - Put your own ROSBAG file inside a newly created 'data' folder and rename it to 'PA7.bag'. Some further changes need to be made in the script to handle the               rostopic names containing right and left camera image.   

1. Download the folder into the 'src' folder of your catkin workspace.  
2. Build the catkin workspace.
3. Execute the command 'roslaunch lab6 lab7.launch' to view the camera calibration matrix of your phone camera (to need to replace already existing images with images      taken by your phone camera), disparity image of the current frame and the RGB-D point cloud in the Rviz tool.
