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

