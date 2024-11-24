# my_gazebo_tutorials

This repo is part is part of one of the excercises in ENPM700: Software Engineering in Robotics to implement a simple walker algorithm similar to a Roomba robot vacuum cleaner.

## Step 1: Create and navigate to your workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

## Step 2: Clone the repository
git clone https://github.com/robosac333/my_gazebo_tutorials.git

## Step 3: Build the workspace
Build for proper clang-tidy search
```sh
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Step 4: Source the setup file
source install/setup.bash

## Instructions to run nodes:
To start the publisher node, use:
```sh
ros2 run walker robot_control
```

## Launching nodes
To spawn the robot in turtlebot world
```sh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
To collect the data for all the running nodes while the robot is navigating the maze and store in a bag file
```sh
ros2 launch walker record_walker.launch.py
```
To launch walker node without recording
```sh
ros2 launch walker record_walker.launch.py record:=false
```

## Use the rosbag
To play the bag file so that the robot executes the same behaviour as it did while recording the bag, download the bag file [here](https://drive.google.com/drive/folders/1hC-qGPnQ-eIoBFeRTpy6EJMPTcrTOjba?usp=sharing) and store it. Go to the folder containing the bag and run the following command
```sh
ros2 bag play walker_recording_2024_11_22-15_56_25
```

## Inspect the bag file

You might replace the path with the complete path to the bag file. This will provide the information about the info inside the rosbag
```sh
ros2 bag info walker_recording_2024_11_22-15_56_25
```

## Google C++ style
To arrange the code as per Google C++ guidlines after making changes.
```sh
clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -v "/build/")
```

## cpp-lint
To check if the code base conforms to cpp-lint style
```sh
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $(find . -name *.cpp | grep -v "/build/")
```

## Clang-tidy
To check if the code is in proper Clang-format go inside the workspace and run the following command
```sh
clang-tidy -p build $( find . -name *.cpp | grep -v "/build/" )
```

## Dependencies

- Ubuntu 22.04 (If running locally)
- ROS2 Humble
- Git
- C++17