# TP3 Lloyd Florens

To run the setup assistant enter below command on a terminal:
- roslaunch moveit_setup_assistant setup_assistant.launch

To launch the package created from the setup assistant run the below command on another terminal:
- roslaunch hand_lloyd demo.launch

To run the direct cinematic service run the below command on a new terminal:
- rosrun udm_hand_control move_hand.py

and to send the command through the server:
rosservice call /udm_service "joint_goal1:
  data: 0.0
joint_goal2:
  data: 0.0
joint_goal3:
  data: 0.0
group:
  data: 'finger_one'" 

To run the launch file:
- roslaunch udm_hand_control cine_direct.launch

To run the inverse cinematic service run the below command on a new terminal:
- rosrun udm_hand_control cineInverse.py

and to send the command through the server:
rosservice call /cinematic_service "orientation: {data: 0.0}
posex: {data: 0.0}
posey: {data: 0.0}
posez: {data: 0.0}
group: {data: 'finger_one'}"

To run the launch file for inverse cinematic:
- roslaunch udm_hand_control cine_inverse.launch






