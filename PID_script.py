#!/usr/bin/env python

###########################################################################
# Author List : Naresh bisht
# Functions : arm, disarm, position_hold, calc_pid, pid_calculator, limit, get_pose, get_yaw
# Global Variables: pluto_cmd, cmd, drone_position, setpoint, kp,ki,kd, correct_roll, correct_pitch, 
#                   correct_throt, correct_yaw, last_time, looptime, previous_error, errors_sum
############################################################################

#The required packages are imported here
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy
import time


class DroneFly():
	"""docstring for DroneFly"""
	def __init__(self): # Class Constructor
		
		#A rosnode named 'pluto_fly' is initilised
		rospy.init_node('pluto_fly', disable_signals = True) 

		#pluto_cmd is a reference variable that can now publish PlutoMsg type of message to the topic '/drone_command' 
		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)


		#below statements makes the node subscribe to the corrosponding topics - to get the drone coordinate and its orientation along its own z axis  
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		rospy.Subscriber('/drone_yaw', Float64, self.get_yaw)   #data_via_rosservice.py script contain code the publish drone orientation to topic /drone_yaw
															                              #using sensors present in drone

		#cmd is an object of PlutoMsg structure and is later used to publish values to topic '/drone_command'
		self.cmd = PlutoMsg()

		#Declaring Global Variables

		# This hold the current position and orientation of drone. the value is updated each time in whycon callback
		# [drone_x, drone_y, drone_z, drone_yaw] i.e index [0] = pitch, [1] = roll, [2] = throttle, [3] = yaw
		self.drone_position = [0.0,0.0,0.0,0.0]
		
		#Position to hold
		self.setpoint = [0, 0, 20.3, 0]
    
		self.Current_error = [0.0,0.0,0.0,0.0]

		
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0
 
		#Coefficients of P,I,D term i.e eg: self.Kp[2] corresponds to Kp value in throttle axis
		self.Kp = [21          ,21       ,45     ,0]  
		self.Ki = [0.6         ,0.2      ,0     ,0]
		self.Kd = [35          ,35       ,3       ,0]

		# Computed values are stored is these variables after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.
		self.correct_throt = 0.0

		# Loop time for PID computation
		self.last_time = 0.0
		self.loop_time = 0.025 	#the time interval after which the pid is computed again

		#previous_error and errors_sum is a list to store previous and sum of errors in the corresponing axis
		self.previous_error = [0.0,0.0,0.0,0.0]  #[pitch_previous_error, roll_previous_error, throttle_preious_error, yaw__pre_error]
		self.errors_sum = [0.0,0.0,0.0,0.0]      #[pitch_errors_sum, roll_errors_sum, throttle_errors_sum, yaw_errors_sum]
												                     #e.g self.previous_error[0] = 0.52 (implies pitch previous error = 0.52)
											                    	 #i.e index [0] = pitch, [1] = roll, [2] = throttle, [3] = yaw

		self.Current_Traversal = 0								 

		#delay of .1 second
		rospy.sleep(.1)


##################################################################################
	# Function Name: arm
	# Input: None
	# Output:  None
	# Logic: #this function is called to arm the drone 
			 #(The condition to arm the drone is rcThrottle = 1000 (minimum value) and rcAUX4 >= 1300.)
	
	# Example Call:	self.arm()
####################################################################################	
	
	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)


###################################################################################
	# Function Name: disarm
	# Input: None
	# Output:  None
	# Logic: #this function is called to disarm the drone 
			 #(The condition to disarm the drone is rcAUX4 <= 1200)
	
	# Example Call:	self.disarm()	
####################################################################################
	
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)


####################################################################################
	# Function Name: position_hold
	# Input: None
	# Output:  None
	# Logic: #this function is called from main() and is one of the important function that call all necessary functions req for pid calc
			 #this function contain while loop that keep executing until the node is turned off 
			 #(i.e calculating the pid till the program runs)
	# Example Call: position_hold()		
####################################################################################
	

	def position_hold(self):

		rospy.sleep(2.3)

		print "START"

		#arming the drone (disarm and arm for best results) 
		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)

		#executing the while loop until the node is turned off (i.e calculating the pid till the program runs)
		while True:

			# if (input() == s):		
			# 	print "Pollination Done! Pollinated 1 Red Daylily, 1 Green Carnation and 1 Blue Delphinium"
			# 	print "Stop"

			
			#function to calcualte pid in all axis 
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
		 	
		 	pitch_value = int(1500 - self.correct_pitch)              #adding/subtracting Computed pid to give drone motion in x direction
			self.cmd.rcPitch = self.limit (pitch_value, 1530, 1470)   #clamping pitch value between 1550 - 1450. 
															
			roll_value = int(1500 + self.correct_roll)				  #adding/subtracting Computed pid to give drone motion in y direction
			self.cmd.rcRoll = self.limit(roll_value, 1525,1475)		  #clamping roll value between 1600 - 1400.
															
			throt_value = int(1500 - self.correct_throt)			  #adding/subtracting Computed pid to give drone motion in z direction
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1250)  #clamping throttle value between 1600 - 1400.

			yaw_value = int(1500 + self.correct_yaw)				  #adding/subtracting Computed pid to make drone stay at a specific orientation
			self.cmd.rcYaw = self.limit(yaw_value, 1700,1300)		  #clamping yaw value between 1600 - 1400.
			
			# self.cmd.rcThrottle = 1820

			self.pluto_cmd.publish(self.cmd)						  #publishing computed pid to topic '/drone_command'
																	  #the vrep subscribe to this topic to apply changes to drone
	
#####################################################################################
	# Function Name: calc_pid
	# Input: None
	# Output: None
	# Logic: calculate pid for all axis after looptime has passed, kit calls pid_calculator
	
	# Example Call: calc_pid()
#######################################################################################


	def calc_pid(self):
		
		#the below if condition ensures the pid is calculated only after a specific time interval (looptime)
		self.seconds = time.time()
		current_time = self.seconds - self.last_time
		if(current_time >= self.loop_time):
			
			self.correct_pitch = self.pid_calculator(0)  #0 is index of pitch
			self.correct_roll = self.pid_calculator(1) * -1	 #1 is index of roll.NOT - ROLL IS MULTIPLIED WITH A -ve PWM FOR CORRECT MOTION
			self.correct_throt = self.pid_calculator(2)  #2 is index of throtle
			self.correct_yaw = self.pid_calculator(3)    #3 is index of yaw
			self.Check_Current_Error()					 #check weather the drone has reached the current set point and if yes, change the destination
														 #to anathor set point coordinates.

			#incresing last time to current time so that above if condition become true only after next looptime interval
			self.last_time = self.seconds	



#################################################################################################
	# Function Name: Check_Current_Error
	# Input: index -> None
	# Output: None
	# Logic: When error in all axis get under an error box of 2 we change the setpoint of the drone.
	
	# Example Call: self.Check_Current_Error()
##################################################################################################	

	def Check_Current_Error(self):

		if(self.Current_Traversal < 10):
			if(self.abs(self.Current_error[2]) <= 1):							#change destination when abs value of current error in z-axis is below 1.3
				if(self.abs(self.Current_error[1]) <= 2):						#change destination when abs value of current error in y-axis is below 0.2
					if(self.abs(self.Current_error[0]) <= 2):					#change destination when abs value of current error in x-axis is below 0.2
			 			self.Current_Traversal = self.Current_Traversal + 1		#increase value of current traversal.
					 	self.change_coordinates(self.Current_Traversal)			#changing coordinates according to current_traversal value.		


#################################################################################################
	# Function Name: pid_calculator
	# Input: index -> (int) pass 0 for calculating pitch, 1 for roll etc.
	# Output: Computed_pid -> (Float64) calculated pid values for axis(roll, pitch etc) whose index(0,1,2,3) was passed 
	# Logic: this func calcualte pid by calcualting Proportional_term, Intergral_term, Derivative_term. if 0 is passed in index then calculations are
	#        performed using kp[0], errors_sum[0], previous_error[0] etc hence pid for pitch is calculated and return cuz kp[0] etc store cooresponding values of pitch 
	
	# Example Call: pid_calculator(0)	
##################################################################################################
	

	def pid_calculator(self, index):

			#calculate current residual error, the drone will reach the desired point when this become zero
			self.Current_error[index] = self.setpoint[index] - self.drone_position[index]      
			
			#calculating values req for finding P,I,D terms. looptime is the time delta_time (dt).
			self.errors_sum[index] =self.errors_sum[index] + self.Current_error[index] * self.loop_time 
			self.errDiff = (self.Current_error[index] - self.previous_error[index]) / self.loop_time

			#calculating individual controller terms - P, I, D.
			self.Proportional_term = self.Kp[index] * self.Current_error[index]
			self.Derivative_term = self.Kd[index] * self.errDiff
			self.Intergral_term = self.Ki[index] * self.errors_sum[index] 

			#computing pid by adding all indiviual terms
			self.Computed_pid = self.Proportional_term + self.Derivative_term + self.Intergral_term 

			#storing current error in previous error after calculation so that it become previous error next time
			self.previous_error[index] = self.Current_error[index]

			#returning Computed pid
			return self.Computed_pid



###########################################################################################################
	# Function Name: limit
	# Input: input_value - (int) the value to clamp
	#        max_value - (int) the max value above which input_value will be clamped
	#        min_value - (int) the min value below which input_value will be clamped
	# Output: None
	# Logic: limiting drone speed to the max and min value	
	#		 (if input_value > max_value - return max_value
	#        if input_value < min_value - return min_value
	#        else return input_value)
	
	# Example Call: limit(1540, 1600, 1400)
############################################################################################################


	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

	def abs(self, value):
		
		if(value < 0):
			value = -value
		return value		

###########################################################################################################
	# Function Name: get_pose
	# Input: pose
	# Output: None
	# Logic: get position of drone according to camera coordiante system using whycon. this is callback function of whycon topic
	
	# Example Call: callback function called automaticallay when whycon coordiantes updates (get_pose(pose))
############################################################################################################	

	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		self.drone_position[0] = pose.poses[0].position.x
		self.drone_position[1] = pose.poses[0].position.y
		self.drone_position[2] = pose.poses[0].position.z

###########################################################################################################
	# Function Name: get_yaw
	# Input: orientation
	# Output: None
	# Logic: get orientation of drone along z axis 
	
	# Example Call: callback function called automaticallay when orienation of drone changes (get_yaw(orientation))
############################################################################################################			

	def get_yaw(self, orientation):
		
		#This is the subscriber function to get the drone orientation.the value is updated each time in /drone_yaw callback 
		self.drone_position[3] = orientation.data		

###########################################################################################################
	# Function Name: change_coordinates
	# Input: None
	# Output: None
	# Logic: Change coordinate of the drone setpoint so that drone when get under an error box move to the next direction. 
	
	# Example Call: self.change_coordinates()
############################################################################################################	


	def change_coordinates(self, Current_Traversal):	
		if(Current_Traversal == 1):
			self.setpoint = [-6.3, -1, 20.8, 0]
		elif(Current_Traversal == 2):
			self.setpoint = [0, 0, 19, 0.0]	
		elif(Current_Traversal == 3):
			self.setpoint = [0, 0, 20.3, 0.0]		
		elif(Current_Traversal == 4):
			self.setpoint = [0.6, -6.7, 21.8	, 0.0]			
		elif(Current_Traversal == 5):
			self.setpoint = [0, -3, 17.6, 0.0]	
		elif(Current_Traversal == 6):
			self.setpoint = [0, -3, 17, 0.0]
		elif(Current_Traversal == 7):
			self.setpoint = [8, -2.5, 17, 0.0]

if __name__ == '__main__':
	while not rospy.is_shutdown():
		DroneFlyObj = DroneFly()     #creating object of class DroneFly and calling constructor
		DroneFlyObj.position_hold()  #calling position_hold function of class Drone_Fly
		rospy.spin()		  #spining so that the main function does get terminated
    
    
    
    #Give me a follow in github and star this repository if u like this script.
    #Happy coding.







