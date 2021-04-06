# Written by FT, 5/4/21

import rospy
from pynput.keyboard import Key, Listener
from geometry_msgs.msg import Twist

def press(key):
	print('\nYou Entered {0}'.format(key))

	msg = Twist()
	msg.linear.x = 0
	msg.linear.y = 0
	msg.angular.z = 0

	if key == Key.up:
		msg.linear.x = vel_x
	elif key == Key.down:
		msg.linear.x = -vel_x
	elif key == key.left:
		msg.linear.y = vel_y
	elif key == key.right:
		msg.linear.y = -vel_y
	elif key == key.page_up:
		msg.angular.z = vel_rot
	elif key == key.page_down:
		msg.angular.z = -vel_rot
	else:
		print('\nKey {0} not a valid command'.format(key))

	pub.publish(msg)

def release(key):
	msg = Twist()
	msg.linear.x = 0
	msg.linear.y = 0
	msg.angular.z = 0

	pub.publish(msg)

# Program params
# Movement velocities
vel_x = 0.2
vel_y = 0.2
vel_rot = 0.2

rospy.init_node("teleoperate")
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Instructions
print('*** Teleoperate ***')
print('up/down = forwards/backwards')
print('right/left = right/left')
print('page down/page up = rotate right/rotate left')

with Listener(on_press=press, on_release=release) as listener:
	listener.join()

#while not rospy.is_shutdown():
#	
#	# Key presses will update these values.
#	# NOTE: This only works for one key at a time.
#	#with Events() as events:
#	#	# Block for at most update_interval_sec seconds:
#	#	event = events.get(0.2)
#	#	if event is not None:
#	#		update(event.key)
#	#	else:
#	#		print('.') #'No key press events')
#	
#	pub.publish(msg)
#	print('Something is happening')
#	rate.sleep()
