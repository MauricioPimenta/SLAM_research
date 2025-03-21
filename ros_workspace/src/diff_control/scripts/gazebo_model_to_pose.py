#!/usr/bin/env python3

# Subscribe to the gazebo model state to get the pose of the robot and publish it to the topic /robot_pose
# The pose is published as a PoseWithCovarianceStamped message
# The covariance is set to 0.001 for the position and 0.001 for the orientation
# The pose is published whenever the model state is updated
# the pose is published in the frame 'map'

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose


class GazeboModelToPose:
	def __init__(self):
		rospy.init_node('gazebo_model_to_pose', anonymous=True)
		self.pose_pub = rospy.Publisher('/robot_pose', PoseWithCovarianceStamped, queue_size=10)
		rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
		self.robot_name = rospy.get_param('~robot_name', 'turtlebot3')  # Default robot name is 'robot'

		# create timer to publish the pose every 1/f seconds
		self.pose_frequency = rospy.get_param('~pose_frequency', 2)
		self.timer = rospy.Timer(rospy.Duration(1/self.pose_frequency), self.publishPoseCallback)

	def model_states_callback(self, data:ModelStates):
		try:
			robot_index = data.name.index(self.robot_name)
			robot_pose = data.pose[robot_index]

			self.pose_msg = PoseWithCovarianceStamped()
			self.pose_msg.header.stamp = rospy.Time.now()
			self.pose_msg.header.frame_id = 'map'
			self.pose_msg.pose = PoseWithCovariance()
			self.pose_msg.pose.pose = robot_pose
			self.pose_msg.pose.covariance = [0.001] * 36  # Set covariance for position and orientation

		except ValueError:
			rospy.logwarn(f"Robot name '{self.robot_name}' not found in Gazebo model states.")

	def publishPoseCallback(self, event):
		rospy.loginfo("Publishing robot pose")

		# Publish the pose
		self.pose_pub.publish(self.pose_msg)

if __name__ == '__main__':
	try:
		node = GazeboModelToPose()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass