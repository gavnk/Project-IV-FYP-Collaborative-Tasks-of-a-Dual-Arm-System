


# Collaborative-Tasks-of-a-Dual-Arm-System

![image](https://github.com/user-attachments/assets/41da1b03-dea8-4a8d-88dc-6535d0d70ffd)

## Description: 
This project uses two `OPENMANIPULATOR-X Robotic Arms` connected to one `OPENCR` dev board to perform cooperative/collaborative tasks, such passing an object from one arm to the other,
or lifting something together. The project also used two `D435i Realseanse Cameras` mounted directly on the arms. Most of the information for the arms can be found on the Robotis website.
The issue with the Robotis Moveit example is it doesn't compensate for the fact that the arms are 4DOF and Moveit can only work with 6DOF. This project solves this issue by adding virtual joints.
The project makes use of the AR Tag `ar_track_alvar` library for object location.

This project was developed on a machine running `Ubuntu 20.04` and `ROS1 Noetic` distro.

A Video of the Project: FYP https://www.youtube.com/@gavinkenny369/videos

For Project Tasks go to directory: `open_manipulator_perceptions/open_manipulator_pick_and_place/src`

**Update: Added class RobotPoses for task 1: Passing Of An Object, run with roslaunch open_manipulator_pick_and_place task1.main.launch,
this includes files robot_poses.h which has all the functions for the different poses.
This makes the design more organized, but for troubleshooting and debugging its recommended to use the task#.cpp files.**


This repo includes all the required software for the robot arms, cameras, and AR tag detection which are all open source software.

Moveit needs 6DOF to work, so 2 virtual joints were added. 
This is a solution to the existing moveit setup for the openmanipulator 
arms to allow for IK movements and control of the end effector and not just the joints.

By running the tasks below, the camera will detect the AR tag markers 1,2 or 3 and start performing the operations.
- Task 1: Passing of an object
- Task 2: Pick up Platform 1
- Task 3: Pick up Platform 2
- Task 4: Custom PID control of shared object

Hardware:
- 2x 4DOF OpenmanipulatorX Robotic Arms by Robotis
- 2x D435i Intel RealSense Depth Cameras
- 1x OPENCR Dev Board
- AR Tags




![image](https://github.com/user-attachments/assets/2ae44f37-f5f1-4ce8-a8e8-e822f5877429)
![image](https://github.com/user-attachments/assets/3ec9e5e6-36ee-4b0e-ba46-477125611656)


![image](https://github.com/user-attachments/assets/9bb4574b-55c6-4fec-8d01-7638aa418193)
![image](https://github.com/user-attachments/assets/88248111-5042-46fb-a5d0-0240c39dfafa)


![image](https://github.com/user-attachments/assets/ef20e3ba-181a-4b75-be30-f5dba8a70092)
![image](https://github.com/user-attachments/assets/4062eefb-2f29-4b98-aaa1-50f128a2e4c5)


Software:
- Install ROS1 Noetic
- Install Linux 20.04
- Install ar tag library, see Robotis website.
- Install realsense software, see https://github.com/IntelRealSense/realsense-ros
  
![image](https://github.com/user-attachments/assets/419bcf2c-86fa-4579-8556-5bb3b565bf9b)

- MoveIt
  


![image](https://github.com/user-attachments/assets/bf5ced25-75cd-4f7e-bf5e-8c754dbaf8c8)

### How to start:

1. Create a ROS workspace: `mkdir project_ws`.
2. `dir project_ws` then `mkdir src`.
3. Source workspace: `source /opt/ros/noetic/setup.bash`
4. Clone repo into `src` folder of your workspace : git clone...
5. Move files in folder named `Collaborative-Tasks-of-a-Dual-Arm-System` folder to `src` folder and delete Collaborative-Tasks-of-a-Dual-Arm-System now empty folder
6. Return to project_ws: `cd ~/project_ws/`
7. Build with: `catkin_make`
8. Source workspace: `source devel/setup.bash`
9. Launch Simulation: `roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=true`.
10. Launch Physical: `roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=false`.
For using the hardware, both arms must be connected to one OPENCR board, and the motor IDs may need to be changed.


![image](https://github.com/user-attachments/assets/6daf9373-3b00-4b50-93c3-1d6c6d57e604)

# Dual Arm Tasks in MoveIt.
For only using with Hardware:
Open another terminal:

### Task 1: Passing of an object: roslaunch open_manipulator_pick_and_place main.launch

![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/bcb84544-4ecf-4d5d-9e43-0100308c13bf)
![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/2d4fb88e-eea5-4db6-9fb7-8665143b945f)

### Task 2: Pick up Platform 1: roslaunch open_manipulator_pick_and_place task2.launch

![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/6748565a-2223-40a8-bcc3-ae9d9ecfc52b)

### Task 3: Pick up Platform 2: roslaunch open_manipulator_pick_and_place task3.launch

![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/cbc746ed-c5ad-4d77-bbe0-d47299beb024)

### Task 4: Manipulation of shared object: roslaunch open_manipulator_pick_and_place task4.launch
* Before launching Task 4, Important: comment out on arm static transform publisher and uncomment off arm stp in ar_pose.launch file in open_manipulator_ar_markers package, this selects the camera setup for on a tripod behind the robots
Task 4: PID control of shared object: roslaunch open_manipulator_pick_and_place task4_PID.launch

![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/2da7e90a-fea0-4217-975d-53ffeb9f9855)
