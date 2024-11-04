# turtle_draw_stuff
A program for ros2 turtlesim where the turtle draws things.\
This program uses a custom file format (.tdraw), which it uses to command the turtle.
## Usage
```ros2 run turtle_draw_stuff turtle_draw --ros-args -p "filename:=[ENTER .tdraw FILE HERE]"```
## Compilation
requirements: ros2-humble, colcon\
Step 1: Go to this repo's root directory using a  terminal\
Step 2: Type in the terminal: 
```colcon build```. This builds the package\
Step 3: To use this in ros2, you must source one of the install "setup" scripts in the install folder. For example, run: ```source install/setup.bash```\
Step 4: You should be all set!
