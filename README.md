# Collaborative-Tasks-of-a-Dual-Arm-System
FYP https://www.youtube.com/@gavinkenny369/videos

Create a ROS workspace: mkdir project_ws

Source workspace: source /opt/ros/noetic/setup.bash

mkdir src

Clone repo into src folder of your workspace.

Move files in folder named 'Collaborative-Tasks-of-a-Dual-Arm-System' folder to src folder and delete Collaborative-Tasks-of-a-Dual-Arm-System folder

return to project_ws: cd ~/project_ws/

Build with Catkin_make

Source workspace: source devel/setup.bash

Launch with: roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=false

Open another terminal:

# Dual Arm Tasks in MoveIt.
Task 1: Passing of an object: roslaunch open_manipulator_pick_and_place task1.launch

![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/bcb84544-4ecf-4d5d-9e43-0100308c13bf)
![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/2d4fb88e-eea5-4db6-9fb7-8665143b945f)

Task 2: Pick up Platform1: roslaunch open_manipulator_pick_and_place task2.launch

![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/6748565a-2223-40a8-bcc3-ae9d9ecfc52b)

Task 3: Pick up Platform2: roslaunch open_manipulator_pick_and_place task3.launch

![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/cbc746ed-c5ad-4d77-bbe0-d47299beb024)

* Before launching Task 4, Important: comment out on arm static transform publisher and uncomment off arm stp in ar_pose.launch file in open_manipulator_ar_markers package, this selects the camera setup for on a tripod behind the robots
Task 4: PID control of shared object: roslaunch open_manipulator_pick_and_place task4_PID.launch

![image](https://github.com/gavnk/Collaborative-Tasks-of-a-Dual-Arm-System/assets/50642905/2da7e90a-fea0-4217-975d-53ffeb9f9855)
