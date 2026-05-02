## Getting Started

If you clone this into new environment then we have to build it again from scratch again. 
Please follow this step

### Step 1 : Delete and build
```
cd ROS1_Dronekit_Indoor

rm -rf build devel

catkin_make

source deve/setup/bash
```

## Step 2  : run our scripts

Choose any of the script that you want to run.
```
jlukas@Techmon:~/My_Project/ROS1/ROS1_DroneKit_Indoor$ rosrun indoor

keyboard_motion_control_field.launch          state.py
keyboard_motion_control_field.py              takeoff_and_loiter_optical_flow_field.launch
keyboard_motion_control_sim.launch            takeoff_and_loiter_optical_flow_field.py
keyboard_motion_control_sim.py                takeoff_and_loiter_optical_flow_sim.launch
simulator.launch                              takeoff_and_loiter_optical_flow_sim.py
start_sim_vehicle.py     ```
