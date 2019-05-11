# Final Project

### The video can be found at this link: https://bit.ly/2E0FslG and the final paper is included in repository 

### The main file to be graded is main.py with modules pulled from move_script.py, cool_math.py and map_script.py

The overall approach to the project was to use the robot’s knowledge of its home EKF-position and the EKF-position of the ARTag to decide its path to a given dispenser. The robot would move toward the EKF-position of the ARTag until it recognized the ARTag and was closed enough to it in order successfully park. This would then activate the parking sequence for the robot, allowing it to geometrically plan the next steps it would have to take to place itself under the candy dispenser and wait for candy to be dispensed. After about ten seconds, the robot would then return to its home position and wait for another command to tell it where to go. To handle the occurrence of obstacles in the robot’s planned path to the EKF-location of the ARTag the robot used the depth sensor. The depth sensor was able to identify objects that were not ARTag within a certain distance of the robot and send a message to the main program to tell the robot that there was an obstacle to be “avoided”. Whether or not the robot would avoid the obstacle was dependent on how far the robot away was from the dispenser. If the robot was quite far from the dispenser, it would pause momentarily before slighting adjusting its direction to veer out of the path that was being blocked by an obstacle. If it was closer to the ARTag, it would pause for a longer amount of time, acting under the assumption that there was another robot in the location of the ARTag.
