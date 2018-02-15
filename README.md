# Project Vino-Blue
Just because that's the best first random word we came up with trying to fit a title to the context! ;D

The aim of this project was to integrate the modalities of the robot to achieve a pointing task by recognizing the objects that the robot was trained to detect.

* The robot waits for human voice commands to interpret about which object to look for, through a speech processing module.
* Once the command is recognized by the robot, it tries to detect the object in its visual field using a pre-trained deep convolutional neural networks (DCNN) trained on a given images dataset. 
* Once this object is detected by the vision cameras, the 3D point of the object with respect to the base of the robot is computed and is passed onto a kinematics module.
* The kinematics module solves the inverse kinematics problem to point at the detected object.
* Once the object is pointed, the robot asks for feedback from the user by making a high-five gesture.
* Based on predefined human interactions, the forces acting on the hand of the robot is detected and compared for a positive or a negative feedback. The positive feedback is when the user hits at the inside of the hand and negative feedback is when the user hits at the back of the hand.
* Depending on the feedback, the robot expresses happiness or sadness through an emotion controller. Then it gets back to the home position. 



### Development guidelines

3 Basic rules
1. Do not commit to master
2. Do not commit to master
3. Do not commit to master

#### Building a module
Create your modules inside module directory. Add `add_subdirectory(dir_name)` to module/CMakeLists.txt

More stuff coming soon...
