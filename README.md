# farscope_project_entry

This ROS package is intended to be a template for "entries" into the [FARSCOPE](https://www.farscope.bris.ac.uk/) Group Project in conjunction with the 
[farscope_group_project](https://github.com/arthurrichards77/farscope_group_project) simulator package.

## Getting started

1. Install and test [farscope_group_project](https://github.com/arthurrichards77/farscope_group_project) and its dependencies using the instructions provided
2. [Fork this repository](https://docs.github.com/en/github/getting-started-with-github/fork-a-repo) to your personal Github account.
3. [Clone](https://docs.github.com/en/github/creating-cloning-and-archiving-repositories/cloning-a-repository) the group's forked repository to
local machines and test it, using `roslaunch farscope_project_entry full_simulation.launch`. 
4. Edit your forked copy of this package to implement your solution to the challenge.

> You are strongly advised to agree early on [how to manage your collaborative coding](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests).  In particular, *agree which is the main repository and branch for your group*.
Then you can either have [shared access to a single fork](https://docs.github.com/en/enterprise-server@3.0/github/setting-up-and-managing-your-github-user-account/inviting-collaborators-to-a-personal-repository),
perhaps each with your own branch, or you can each have [different forks](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/working-with-forks).

> Please do not submit pull requests back to this repository - it is intended to serve as a template.  Feel free to report issues with the template though.

## Run the program

The first step is downloading the weights for the neural network from https://drive.google.com/file/d/1ESFDo8f49XlGB-oBHgf-u93HxaLl8cJI/view?usp=sharing. Afterwards the file needs to be placed inside 'farscope_project_entry/scripts/perception/trophy_detector'.

The program can then be run by executing 'roslaunch farscope_project_entry integration.launch'.

## Controlling the robot

In this section, we walk through the [`our_controller.py`](https://github.com/arthurrichards77/farscope_project_entry/blob/main/scripts/our_controller.py) example file.
It starts by importing `rospy` to talk ROS from Python and the three main interface classes `farscope_robot_utils` package from `farscope_group_project`, which wraps the ROS interfaces of the robot components.
```python
#!/usr/bin/env python
import rospy
from farscope_group_project.farscope_robot_utils import ArmMover, BaseDriver, GripperController
```
Next the program initializes itself as a ROS node and creates the three interfaces to the robot.
```python
rospy.init_node("our_controller")

base_driver = BaseDriver()
arm_mover = ArmMover()
gripper_controller = GripperController()
```
It can take a while for the trophy spawning process to complete when the simulator starts up, so to avoid chasing targets that aren't there yet, the program
waits for the `target_spawning_complete` parameter that is set by the spawning script.
```python
# wait for parameter that says spawning is finished
rospy.loginfo("Waiting for spawning to finish")
rate = rospy.Rate(2.0)
while not rospy.get_param('target_spawning_complete',False):
    rate.sleep()
rospy.loginfo("Spawning completion detected - time to get going!")
```
Now the action starts - open the gripper and move the arm to a hard-coded position aligned with the second shelf.  I found this by trial and error, using the `move_arm` utility provided with `farscope_group_project`. 
```python
# open gripper ready
gripper_controller.open()
# lift arm to what seems to be right height
arm_mover.move(shoulder_lift_cmd_in=-0.6, elbow_cmd_in=1.0)
```
That blocks until the arm is in position.  Next, the base makes two moves: the first aligns it with the target at the shelf centre, and the second drives the gripper around the target itself.
These were again found by trial and error using the `move_base.py` utility program.
```python
# drive on to the target
base_driver.move(0.3, -0.1)
base_driver.move(0.3, -0.05)
```
Now the gripper is closed, hopefully (this is all open loop) grasping the target.  Then we raise the arm slightly to lift the target off the shelf.
```python
# grab it
gripper_controller.close()
# lift it off shelf
arm_mover.move(shoulder_lift_cmd_in=-0.7, elbow_cmd_in=1.0)
```
The base backs up, turns, and moves forward to leave the target hanging over the drop-off bin. 
```python
# back away
base_driver.move(-0.3, 0, 0, 2)
# turn base to get target over the receptacle
base_driver.move(0, 0, 0.2, 6)
# forward a bit
base_driver.move(0.1)
```
Finally the gripper is opened and the base backs up abruptly to shake the target loose.
```python
# release gripper
gripper_controller.open()
# little nudge to drop it
base_driver.move(-0.5, 0, 0, 0.1)
```

## Extending the software

This section walks through the main `launch` file to explain how to add your own ROS nodes to the control system.  Launch files use [an XML format](http://wiki.ros.org/roslaunch/XML) to set parameters and start nodes, replacing countless `rosrun` commands in multiple terminals with a single `roslaunch` command to bring up a complicated network of interacting programs.  Here, the [`full_simulation.launch`](https://github.com/arthurrichards77/farscope_project_entry/blob/main/launch/full_simulation.launch) file brings up the entire system, including the simulation and the control system.

Every launch file starts with the same preamble and opens a `<launch>` element.
```xml
<?xml version="1.0" ?>

<launch>
```
> You probably don't need to change this next bit.

Launch files can take [arguments](http://wiki.ros.org/roslaunch/XML#substitution_args) to handle things like finding package directories and command line arguments, so one launch file can do different things.  In this next section, two command line arguments are defined, so you can change the scenario file and turn the Gazebo GUI on and off.  They both have default settings, so you can ignore them, or put the argument in to override the default.  For example, `roslaunch farscope_project_entry use_gui:=false` will not start the Gazebo GUI, saving you some computing, and `roslaunch farscope_project_entry full_simulation.launch scenario_file:=/home/<username>/<ros_ws>/src/farscope_group_project/scenarios/all_no_random.yaml` will switch the scenario to use one provided in `farscope_group_project` that puts one trophy in the exact centre of every shelf.  You can use both arguments together.

> Feel free to create your own scenario files for testing.  Commented examples can be found in the [`scenarios` folder](https://github.com/arthurrichards77/farscope_project_entry/tree/main/scenarios).

In the launch file, these arguments are passed straight to the `simulator_only` launch file via an `include` tag.

```xml
  <arg name="scenario_file" default="$(find farscope_project_entry)/scenarios/all_duplicates.yaml" />
  <arg name="use_gui" default="true" />
  <include file="$(find farscope_project_entry)/launch/simulator_only.launch">
    <arg name="use_gui" value="$(arg use_gui)" />
    <arg name="scenario_file" value="$(arg scenario_file)" />
  </include>
```
> You would only need to change the `simulator_only` launch file if you changed the filename of the robot description from `our_robot.urdf.xacro`.

The rest of the launch file is all yours!  In the template, all it does is run the `our_controller.py` script described above.

```xml
  <!-- run example controller -->
  <node name="robot_controller" pkg="farscope_project_entry" type="our_controller.py" />

</launch>
```
You can remove this and replace it with many different interacting nodes to implement your control system.  They can be our own custom-written nodes living in the `farscope_project_entry` package or off-the-shelf nodes from the many hundreds of [available ROS packages](https://index.ros.org/packages/#noetic).  You can start nodes directly or include additional launch files.  Use arguments or [parameters](http://wiki.ros.org/roslaunch/XML/param) to customize node behaviours and [`<remap>` tags](http://wiki.ros.org/roslaunch/XML/remap) to redirect their interconnections.

> When you write your own custom Python scripts, you'll need to [mention them in the package CMakeLists.txt file](http://wiki.ros.org/catkin/CMakeLists.txt#Optional_Step:_Specifying_Installable_Targets) and run a catkin_make before you can use them.  You can also write your own [custom nodes in C++](http://wiki.ros.org/roscpp/Tutorials) if you're so inclined.

## Customizing the robot

The robot is defined by the file [`our_robot.urdf.xacro`](https://github.com/arthurrichards77/farscope_project_entry/blob/main/models/our_robot.urdf.xacro) in the `models` subdirectory.  The robot is represented in [Unified Robot Description Format (URDF)](http://wiki.ros.org/urdf/Tutorials), which is a special schema in XML, and the file uses the [xacro](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File) macro format, _i.e._ the `.urdf.xacro` file must be pre-processed by the `xacro` program to generate a valid URDF description.  You don't need to do this yourself: it is done when the file is loaded into the ROS `robot_description` parameter in the [simulator launch file](https://github.com/arthurrichards77/farscope_project_entry/blob/main/launch/simulator_only.launch).  Xacro enables the model to include other models and to use macros when repeating elements, _e.g._ multiple cameras.  While "pure" URDF only covers the physical description of the robot, our model includes [extra `<gazebo>` elements](http://gazebosim.org/tutorials?tut=ros_gzplugins) that provide sensors and actuators in the simulation.

The file starts with a standard preamble to declare it as a xacro file and give the robot a name.
```xml
<?xml version="1.0"?>
<robot name="our_robot" xmlns:xacro="http://ros.org/wiki/xacro">
```
The mobile manipulator, _i.e._ the arm and the wheeled base, are provided by the `mobile_arm` macro in the `farscope_group_project` package.  First we include the file that defines the macro, and then we run the macro to create the base URDF.  There should be no need to change any of this bit.
```xml
  <!-- mobile base -->
  <xacro:include filename="$(find farscope_group_project)/models/mobile_arm/mobile_arm.urdf.xacro" />

  <xacro:mobile_arm />
```
The gripper is also a macro from the `farscope_group_project` package, so it must be included before it is used.

> Anything that isn't in a `xacro` tag will just pass straight through the `xacro` processing.  So, if you want to develop a different gripper, or indeed add extra stuff to the robot, you can just delete this section and replace it with your own custom URDF.  *Warning:* adding static links with fixed joints is easy enough, but the learning curve to get your own custom model to actually move in Gazebo is rather steep, requiring an understanding of [`ros_control`](http://gazebosim.org/tutorials/?tut=ros_control) and getting URDF, YAML and launch files all to work together. 

```xml
  <!-- ****** gripper ****** -->
  <xacro:include filename="$(find farscope_group_project)/models/gripper/simple_gripper.urdf.xacro" />
```
The `simple_gripper` macro takes two arguments.  The first is a string attribute `parent_link` that specifies to which part of the robot it is attached.  To learn the available links, run the `visualize_robot` roslaunch and inspect the `links` section of the Robot Model item in RViz.  In this initial case, its attached to the `tool0` link which is the very end of the arm, logically enough.  The second argument is an XML `origin` block that defines where on that link it's attached.  You could edit this to offset or rotate the gripper - without the pain of having to produce an actual mount!  URDF will regidly attach it through thin air.
```xml
  <xacro:simple_gripper parent_link="tool0">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
  </xacro:simple_gripper>
```
Finally the example includes a camera using yet another macro from `farscope_group_project`.
```xml
  <!-- ****** cameras ****** -->
  <xacro:include filename="$(find farscope_group_project)/models/camera/simple_camera.urdf.xacro" />
```
Again, we specify a `parent_link` attribute and an `origin` block, which in this example mounts the camera with an offset.
```xml
  <!-- example: camera attached right above gripper -->
  <xacro:simple_camera camera_name="camera1" parent_link="tool0" >
    <origin xyz="0 0.1 0" rpy="0 -1.5706 0" />
  </xacro:simple_camera>
```
Feel free to mess around with the camera position.  You can affix multiple cameras by just duplicating the macro.  Be sure to give each camera a different name, so it will appear on its own ROS topic later.

> You could also use custom URDF here to [add your own extra sensors](http://gazebosim.org/tutorials?tut=add_laser).

Finally the model concludes by ending the `robot` element.  Job done.
```xml
</robot>
```
