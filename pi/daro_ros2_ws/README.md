## build
colcon build

## build specific packages
colcon build --packages-select my_package

colcon build --packages-select 


## symlink build
- Allows you to edit file and that file is in the install folder.
- you just have to edit file, restart node, done. Don't have to rebuild everytime you make a change to config or .py file.

colcon build --symlink-install


## always run from project root
source ./install/setup.bash


## To make daro come to life run this
ros2 launch daro_bringup daro.launch.py
