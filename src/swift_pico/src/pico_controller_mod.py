#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error			/throttle_pid
						/pitch_pid
						/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries
from install.swift_pico.lib.swift_pico import waypoint_service
from tf_transformations import euler_from_quaternion
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from waypoint_navigation.action import NavToWaypoint
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from pid_msg.msg import PIDTune, PIDError
from actuator_msgs.msg import Actuators
import rclpy
from rclpy.node import Node
import numpy as np
import os, csv, time


class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.lqr_callback_group = ReentrantCallbackGroup()
		self.action_callback_group = ReentrantCallbackGroup()

		self.time_inside_sphere = 0
		self.max_time_inside_sphere = 0
		self.point_in_sphere_start_time = None
		self.duration = 0

		self.drone_position = [0.0, 0.0, 0.0]
		self.drone_orientation = [0.0, 0.0, 0.0]
		self.drone_velocity = [0.0,0.0,0.0]
		self.angular_velocity = [0.0,0.0,0.0]
		self.dtime = 0
		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [3, 2, 5]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		# Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		#-----------------------Add other required variables for pid here ----------------------------------------------
		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
		self.sample_time = 0.06  # in seconds

		# Publishing /drone_command, /pid_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
		self.command_rotors = self.create_publisher(Actuators, '/rotors/command/motor_speed', 10)

		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------
	

		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(Odometry, "/rotors/odometry", self.odometry_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)

		#------------------------Add other ROS Subscribers here-----------------------------------------------------
		self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)
		self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
		self.pid_error = PIDError()  # Initialize PIDError message

		# Create or open CSV file
		self.csv_path = os.path.expanduser('~/drone_positions.csv')
		self.csv_file = open(self.csv_path, 'w', newline='')
		self.csv_writer = csv.writer(self.csv_file)
		self.csv_writer.writerow(['time', 'x', 'y', 'z'])  # CSV header

		self.action_server = ActionServer(self, NavToWaypoint, 'waypoint_navigation', self.execute_callback, callback_group=self.action_callback_group)

		self.arm()  # ARMING THE DRONE

		# Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)
		self.timer = self.create_timer(self.sample_time, self.lqr, callback_group=self.lqr_callback_group)


	def disarm(self):
		self.cmd.rc_roll = 1000
		self.cmd.rc_yaw = 1000
		self.cmd.rc_pitch = 1000
		self.cmd.rc_throttle = 1000
		self.cmd.rc_aux4 = 1000
		self.command_pub.publish(self.cmd)
		

	def arm(self):
		self.disarm()
		self.cmd.rc_roll = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_throttle = 1500
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)  # Publishing /drone_command


	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def odometry_callback(self, msg):
		t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
		self.drone_position[0] = msg.pose.pose.position.x 
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.pose.pose.position.y 
		self.drone_position[2] = msg.pose.pose.position.z 

		self.drone_velocity[0] = msg.twist.twist.linear.x
		self.drone_velocity[1] = msg.twist.twist.linear.y
		self.drone_velocity[2] = msg.twist.twist.linear.z

		self.angular_velocity[0] = msg.twist.twist.angular.x
		self.angular_velocity[1] = msg.twist.twist.angular.y
		self.angular_velocity[2] = msg.twist.twist.angular.z

		quaternion = [
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w,
		]

		roll, pitch, yaw = euler_from_quaternion(quaternion)

		self.drone_orientation[0] = roll
		self.drone_orientation[1] = pitch
		self.drone_orientation[2] = yaw

		self.csv_writer.writerow([t, self.drone_position[0], self.drone_position[1], self.drone_position[2]])

		#self.get_logger().info(f"{self.drone_position}")
		#---------------------------------------------------------------------------------------------------------------


	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.ki * 0.008
		self.Kd[2] = alt.kd * 0.6

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.kp * 0.03
		self.Ki[1] = pitch.ki * 0.008
		self.Kd[1] = pitch.kd * 0.6

	def roll_set_pid(self, roll):
		self.Kp[0] = roll.kp * 0.03
		self.Ki[0] = roll.ki * 0.008
		self.Kd[0] = roll.kd * 0.6
	#----------------------------------------------------------------------------------------------------------------------
	def map_u_to_pwm(self, u):
		# u[0] = total thrust, u[1] = roll moment, u[2] = pitch moment, u[3] = yaw moment
		pwm_base = 660
		pwm = np.zeros(4)

		pwm[0] = pwm_base + (u[0])
		pwm[1] = pwm_base + (u[1])
		pwm[2] = pwm_base + (u[2])
		pwm[3] = pwm_base + (u[3])

		return pwm


	def lqr(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
		x_state = np.array([
			self.drone_position[0], self.drone_velocity[0],
			self.drone_position[1], self.drone_velocity[1],
			self.drone_position[2], self.drone_velocity[2],
			self.drone_orientation[0],    self.angular_velocity[0],
			self.drone_orientation[1],    self.angular_velocity[1],
			self.drone_orientation[2],    self.angular_velocity[2]
    	])
		x_desired = [
			self.setpoint[0], 0,          # desired x and x_dot
			self.setpoint[1], 0,          # desired y and y_dot
			self.setpoint[2], 0,          # desired z and z_dot
			0, 0,                   # desired phi and p
			0, 0,                   # desired theta and q
			0, 0                    # desired psi and r
		]

		'''
		K = np.array([
			[-5.0000e+00, -1.2963e+01,  5.0000e+00,  8.2698e+00,  5.0000e+00,  1.9070e+01,  4.2277e+01,  1.1743e+01, -1.3937e+02, -6.4395e+01,  1.1040e-07, -5.9367e-04],
			[ 5.0000e+00,  1.2963e+01, -5.0000e+00, -8.2698e+00,  5.0000e+00,  1.9070e+01, -4.2277e+01, -1.1743e+01,  1.3937e+02,  6.4395e+01, -1.1040e-07,  5.9367e-04],
			[-5.0000e+00, -1.2963e+01, -5.0000e+00, -8.2698e+00,  5.0000e+00,  1.9070e+01, -4.2277e+01, -1.1743e+01, -1.3937e+02, -6.6427e+01, -1.7771e-05, -1.7637e-01],
			[ 5.0000e+00,  1.2963e+01,  5.0000e+00,  8.2698e+00,  5.0000e+00,  1.9070e+01,  4.2277e+01,  1.1743e+01,  1.3937e+02,  6.6427e+01,  1.7771e-05,  1.7637e-01]
		])
		'''
		#improved#1
		'''K = np.array([
			[-5.0000e+00, -1.4418e+01,  5.0000e+00,  8.5633e+00,  5.0000e+00,  1.9070e+01,  4.7091e+01,  1.4241e+01, -1.7820e+02, -9.3277e+01, -1.4353e-05, -1.8291e-01],
			[ 5.0000e+00,  1.4418e+01, -5.0000e+00, -8.5633e+00,  5.0000e+00,  1.9070e+01, -4.7091e+01, -1.4241e+01,  1.7820e+02,  9.3277e+01,  1.4353e-05,  2.0244e-01],
			[-5.0000e+00, -1.4418e+01, -5.0000e+00, -8.5633e+00,  5.0000e+00,  1.9070e+01, -4.7091e+01, -1.4241e+01, -1.7820e+02, -8.7340e+01,  1.5449e-05,  1.6865e-01],
			[ 5.0000e+00,  1.4418e+01,  5.0000e+00,  8.5633e+00,  5.0000e+00,  1.9070e+01,  4.7091e+01,  1.4241e+01,  1.7820e+02,  8.7340e+01, -1.5449e-05, -1.6865e-01]
		])'''

		#improved#2
		'''K = np.array([
			[-5.0000e+00, -1.4418e+01,  5.0000e+00,  8.5633e+00,  3.5355e+00,  1.6263e+01,  4.7091e+01,  1.4241e+01, -1.7820e+02, -8.5186e+01,  2.6142e-05,  2.9021e-01],
			[ 5.0000e+00,  1.4418e+01, -5.0000e+00, -8.5633e+00,  3.5355e+00,  1.6263e+01, -4.7091e+01, -1.4241e+01,  1.7820e+02,  8.5186e+01, -2.6142e-05, -2.9021e-01],
			[-5.0000e+00, -1.4418e+01, -5.0000e+00, -8.5633e+00,  3.5355e+00,  1.6263e+01, -4.7091e+01, -1.4241e+01, -1.7820e+02, -8.8936e+01,  7.0684e-06,  7.5362e-02],
			[ 5.0000e+00,  1.4418e+01,  5.0000e+00,  8.5633e+00,  3.5355e+00,  1.6263e+01,  4.7091e+01,  1.4241e+01,  1.7820e+02,  8.8936e+01, -7.0684e-06, -7.5362e-02]
		])'''
		
		#improved with drag coefficients
		'''K = np.array([
		    [-5.0000e+00, -1.4418e+01,  5.0000e+00,  8.5632e+00,  3.5355e+00,  1.6263e+01,  4.7091e+01,  1.4241e+01, -1.7820e+02, -9.1624e+01, -4.8583e-06, -9.0328e-02],
		    [ 5.0000e+00,  1.4418e+01, -5.0000e+00, -8.5632e+00,  3.5355e+00,  1.6263e+01, -4.7091e+01, -1.4241e+01,  1.7820e+02,  9.1624e+01,  6.0504e-06,  9.0328e-02],
		    [-5.0000e+00, -1.4418e+01, -5.0000e+00, -8.5632e+00,  3.5355e+00,  1.6263e+01, -4.7091e+01, -1.4241e+01, -1.7820e+02, -9.1311e+01, -3.6662e-06, -7.0797e-02],
		    [ 5.0000e+00,  1.4418e+01,  5.0000e+00,  8.5632e+00,  3.5355e+00,  1.6263e+01,  4.7091e+01,  1.4241e+01,  1.7820e+02,  9.1311e+01,  3.6662e-06,  7.0797e-02]
		])'''

		import numpy as np

		K = np.array([
			[-4.99999996e+00, -1.44184544e+01, -5.00000000e+00, -8.56329242e+00, 5.00000000e+00,  1.90704202e+01,  4.70912034e+01,  1.42413634e+01, -1.78204020e+02, -9.02047279e+01, -9.73234958e-09, -6.10503255e-07],
			[ 4.99999996e+00,  1.44184544e+01,  5.00000000e+00,  8.56329242e+00, 5.00000000e+00,  1.90704202e+01, -4.70912034e+01, -1.42413634e+01, 1.78204017e+02,  9.02047221e+01, -1.72716977e-07,  2.67776548e-07],
			[-4.99999996e+00, -1.44184544e+01,  5.00000000e+00,  8.56329242e+00, 5.00000000e+00,  1.90704202e+01, -4.70912034e+01, -1.42413634e+01, -1.78204012e+02, -9.02047281e+01,  4.82749738e-07, -6.40305578e-07],
			[ 4.99999996e+00,  1.44184544e+01, -5.00000000e+00, -8.56329242e+00, 5.00000000e+00,  1.90704202e+01,  4.70912034e+01,  1.42413634e+01, 1.78204009e+02,  9.02047224e+01, -6.65199050e-07,  2.97578870e-07]
		])


		x_error = x_state - x_desired
		u = -K @ x_error

		self.pid_error.yaw_error = x_error[10]
		self.pid_error.throttle_error = x_error[4]
		self.pid_error.pitch_error = x_error[2]
		self.pid_error.roll_error = x_error[0]
		
		motor_commands = self.map_u_to_pwm(u)

		# Clip commands to [500, 1000]
		motor_commands = np.clip(motor_commands, 400, 900)

		msg = Actuators()
		msg.velocity = motor_commands.tolist()
		#self.get_logger().info(f"{msg}")
	#------------------------------------------------------------------------------------------------------------------------
		self.command_rotors.publish(msg)
		self.pid_error_pub.publish(self.pid_error)
		# calculate throttle error, pitch error and roll error, then publish it accordingly
		#self.pid_error_pub.publish(self.pid_error)

	def execute_callback(self, goal_handle):
		self.get_logger().info('Executing goal...')

		self.setpoint[0] = goal_handle.request.waypoint.position.x
		self.setpoint[1] = goal_handle.request.waypoint.position.y
		self.setpoint[2] = goal_handle.request.waypoint.position.z
		self.get_logger().info(f'New Waypoint Set: {self.setpoint}')

		self.max_time_inside_sphere = 0
		self.point_in_sphere_start_time = None
		self.time_inside_sphere = 0
		self.duration = self.dtime

		#create a NavToWaypoint feedback object. Refer to Writing an action server and client (Python) in ROS 2 tutorials.
		feedback_msg = NavToWaypoint.Feedback()
		#--------The script given below checks whether you are hovering at each of the waypoints(goals) for max of 3s---------#
		# This will help you to analyse the drone behaviour and help you to tune the PID better.

		while True:
			feedback_msg.current_waypoint.pose.position.x = self.drone_position[0]
			feedback_msg.current_waypoint.pose.position.y = self.drone_position[1]
			feedback_msg.current_waypoint.pose.position.z = self.drone_position[2]
			feedback_msg.current_waypoint.header.stamp.sec = (int)(self.max_time_inside_sphere)

			self.dtime = (int)(time.time())

			goal_handle.publish_feedback(feedback_msg)

			drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle, 0.4) #the value '0.4' is the error range in the whycon coordinates that will be used for grading. 
			#You can use greater values initially and then move towards the value '0.4'. This will help you to check whether your waypoint navigation is working properly. 

			if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
				pass
			
			elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
				self.point_in_sphere_start_time = self.dtime
				self.get_logger().info('Drone in sphere for 1st time')                        #you can choose to comment this out to get a better look at other logs

			elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
				self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
				self.get_logger().info('Drone in sphere')                                     #you can choose to comment this out to get a better look at other logs
								
			elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
				self.get_logger().info('Drone out of sphere')                                 #you can choose to comment this out to get a better look at other logs
				self.point_in_sphere_start_time = None

			if self.time_inside_sphere > self.max_time_inside_sphere:
					self.max_time_inside_sphere = self.time_inside_sphere

			if self.max_time_inside_sphere >= 1:
				#self.get_logger().info('Drone has stabilized wooohhhhhhhhhhhhhhh!!')
				break
						

		goal_handle.succeed()

		#create a NavToWaypoint result object. Refer to Writing an action server and client (Python) in ROS 2 tutorials
		result = NavToWaypoint.Result()
		result.hov_time = self.dtime - self.duration #this is the total time taken by the drone in trying to stabilize at a point
		return result

	def is_drone_in_sphere(self, drone_pos, sphere_center, radius):
		return (
			(drone_pos[0] - sphere_center.request.waypoint.position.x) ** 2
			+ (drone_pos[1] - sphere_center.request.waypoint.position.y) ** 2
			+ (drone_pos[2] - sphere_center.request.waypoint.position.z) ** 2
		) <= radius**2

def main(args=None):
	rclpy.init(args=args)
	swift_pico = Swift_Pico()
	executor = MultiThreadedExecutor()
	executor.add_node(swift_pico)
	try:
		executor.spin()
	except KeyboardInterrupt:
		waypoint_service.get_logger().info('KeyboardInterrupt, shutting down.\n')
	finally:
		waypoint_service.destroy_node()
		rclpy.shutdown()
	swift_pico.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
