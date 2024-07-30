# ros2_study
first study for ros2


## rosdep testing
rosdep install -i --from-path src --rosdistro humble -y

## build by colcon
colcon build

## build by colcon - select package
colcon build --packages-select <package_name>

## make source
source install/setup.bash

## run ros executable file 
ros2 run <package_name> <executable_name>

