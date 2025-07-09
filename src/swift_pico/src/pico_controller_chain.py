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
from tf_transformations import euler_from_quaternion
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
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

		self.pwm = np.zeros(4)

		self.drone_position = [0.0, 0.0, 0.0]
		self.drone_orientation = [0.0, 0.0, 0.0]
		self.drone_velocity = [0.0,0.0,0.0]
		self.angular_velocity = [0.0,0.0,0.0]
		self.dtime = 0
		self.u = np.zeros(4)  # This is the control output vector, where u[0] = total thrust, u[1] = roll moment, u[2] = pitch moment, u[3] = yaw moment
		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [3, 2, 4]

		# Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
		self.sample_time = 0.06  # in seconds

		# Publishing /drone_command, /pid_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
		self.command_rotors = self.create_publisher(Actuators, '/rotors/command/motor_speed', 10)

		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------


		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(Odometry, "/rotors/odometry", self.odometry_callback, 1, callback_group=self.lqr_callback_group)
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
	#------------------------------------------------------------------------------------------------------------------------
		#self.command_rotors.publish(msg)

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
		self.cmd.rc_throttle = 1717 # This value varies with payload attatched to the drone.
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)  # Publishing /drone_command

	def odometry_callback(self, msg):
		t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

		#Getting the position of the drone
		self.drone_position[0] = msg.pose.pose.position.x 
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.pose.pose.position.y 
		self.drone_position[2] = msg.pose.pose.position.z 

		#Getting the linear velocity of the drone
		self.drone_velocity[0] = msg.twist.twist.linear.x
		self.drone_velocity[1] = msg.twist.twist.linear.y
		self.drone_velocity[2] = msg.twist.twist.linear.z

		#Getting the angular velocity of the drone
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

		# Getting the orientation of the drone in roll, pitch, yaw
		self.drone_orientation[0] = roll
		self.drone_orientation[1] = pitch
		self.drone_orientation[2] = yaw

		# To get the positions of the drone in csv file
		self.csv_writer.writerow([t, self.drone_position[0], self.drone_position[1], self.drone_position[2]])
		#---------------------------------------------------------------------------------------------------------------


	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.ki * 0.008
		self.Kd[2] = alt.kd * 0.6

	#----------------------------Define callback function like altitude_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.kp * 0.03
		self.Ki[1] = pitch.ki * 0.008
		self.Kd[1] = pitch.kd * 0.6

	def roll_set_pid(self, roll):
		self.Kp[0] = roll.kp * 0.03
		self.Ki[0] = roll.ki * 0.008
		self.Kd[0] = roll.kd * 0.6
	#----------------------------------------------------------------------------------------------------------------------
	def map_u_to_pwm(self):
		pwm_base = 770#765# This is the base PWM value for the motors

		self.pwm[0] = np.clip(pwm_base + (self.u[0]), 500, 800)  # Ensure the PWM values are within a valid range
		self.pwm[1] = np.clip(pwm_base + (self.u[1]), 500, 800)
		self.pwm[2] = np.clip(pwm_base + (self.u[2]), 500, 800)
		self.pwm[3] = np.clip(pwm_base + (self.u[3]), 500, 800)


		#msg = RollPitchYawrateThrust()
		msg = Actuators()
		msg.velocity = self.pwm.tolist()
		self.get_logger().info(f"{msg.velocity}")
	#------------------------------------------------------------------------------------------------------------------------
		self.command_rotors.publish(msg)

	def lqr(self):
	#-----------------------------Write the LQR algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error for each setpoint. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative).
	#	3. Calculate the lqr output by multilying the error with the gain matrix K. For eg: self.out_roll = K[0][0] * error[0] + K[0][1] * error[1] + ... + K[0][n] * error[n], where n is the number of states.
	#	4. Reduce or add this computed output value on the avg value ie 660 or higher depending on the payload attatched.
	#	5. Don't run the lqr continously. Run the lqr only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Add error_sum
	#	7. Publish the command to rotors/command/motor_speed topic
	#	8. Publish the lqr error to /pid_error topic
	#------------------------------------------------------------------------------------------------------------------------

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
			0, 0,              # desired psi and r
		]


		#improved#2
		K = np.array([
			[ -5.        ,  -9.33207242,   5.        ,   8.56329242,
				5.        ,  19.07042021, -47.09120339, -14.24136338,
				-60.49617314, -22.02431898,  -5.        ,  -5.22618154],
			[  5.        ,   9.33207242,  -5.        ,  -8.56329242,
				5.        ,  19.07042021,  47.09120339,  14.24136338,
				60.49617314,  22.02431898,  -5.        ,  -5.22618154],
			[ -5.        ,  -9.33207242,  -5.        ,  -8.56329242,
				5.        ,  19.07042021,  47.09120339,  14.24136338,
				-60.49617314, -22.02431898,   5.        ,   5.22618154],
			[  5.        ,   9.33207242,   5.        ,   8.56329242,
				5.        ,  19.07042021, -47.09120339, -14.24136338,
				60.49617314,  22.02431898,   5.        ,   5.22618154]
				])

		x_error = x_state - x_desired

		self.pid_error.throttle_error = x_error[4]  # z error
		self.pid_error.pitch_error = x_error[2]     # y error
		self.pid_error.roll_error = x_error[0]      # x error

		self.u = -K @ x_error
		
		#self.get_logger().info(f"Control Output: {u}")
		self.map_u_to_pwm()

		# calculate throttle error, pitch error and roll error, then publish it accordingly
		self.pid_error_pub.publish(self.pid_error)

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
	waypoint_server = Swift_Pico()
	executor = MultiThreadedExecutor()
	executor.add_node(waypoint_server)
	try:
		executor.spin()
	except KeyboardInterrupt:
		waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
	finally:
		waypoint_server.destroy_node()
		rclpy.shutdown()
	waypoint_server.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()