#!/usr/bin/env python
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
import rospy
import time
class Edrone():
	def __init__(self):
		rospy.init_node('position_controller_faisal')
		self.errorX = 0
		self.errorY = 0
		self.errorZ = 0
		self.prev_error = [0, 0, 0]
		self.Iterm = [0, 0, 0]
		self.x_error = 0.0
		self.y_error = 0.0
		self.z_error = 0.0

		self.setpoint =0

                

		self.set_latitude = 19.0000451704
		self.set_longitude = 0
		self.set_altitude = 3
		
		self.Kp = [15, 48480, 0]
		self.Ki = [0, 0.8, 0]
		self.Kd = [455, 321230, 0]
		
		self.latitude = 0
		self.longitude = 0
		self.altitude = 0

		self.sample_time = 15

		#self.z_pub = error_z()
		#self.z_pub.zerror = 0

		#self.x_pub = error_x()
		#self.x_pub.xerror = self.set_latitude


		self.rc_pub = edrone_cmd()
		self.rc_pub.rcThrottle = 1000
		self.rc_pub.rcRoll = 1500
		self.rc_pub.rcPitch = 1500
		self.rc_pub.rcYaw = 1500

		self.rc_pos_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

		#self.z_err_pub = rospy.Publisher('/zerror', error_z, queue_size=1)
		#self.x_err_pub = rospy.Publisher('/xerror', error_x, queue_size=1)

		rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.throttle_pid)
		rospy.Subscriber('/lpid_params', PidTune, self.pitch_pid)
		rospy.Subscriber('/rpid_params', PidTune, self.roll_pid)
	def drone_gps_callback(self, msg):
		self.latitude = msg.latitude
		self.longitude = msg.longitude
		self.altitude = msg.altitude
		#print(msg.altitude)
	
	def throttle_pid(self, throttle):
		self.Kp[0] = throttle.Kp * 0.06
		self.Ki[0] = throttle.Ki * 0.008
		self.Kd[0] = throttle.Kd * 0.3 

	def pitch_pid(self, pitch):
		self.Kp[1] = pitch.Kp * 15
		self.Kp[1] = pitch.Kp * 30
		self.Ki[1] = pitch.Ki * 0.8 
		self.Kd[1] = pitch.Kd * 40
		self.Kd[1] = pitch.Kd * 300

	def roll_pid(self, roll):
		self.Kp[2] = roll.Kp * 0.06
		self.Ki[2] = roll.Ki * 0.008 
		self.Kd[2] = roll.Kd * 0.3
	def pos_control(self):

                self.errorZ = (self.set_altitude - self.altitude)
		self.Iterm[0] = (self.Iterm[0] + self.errorZ)*self.Ki[0]
		self.z_error = self.Kp[0]*self.errorZ + self.Kd[0]*(self.errorZ - self.prev_error[0]) +self.Iterm[0]
		self.prev_error[0] = self.errorZ

	        self.errorX = (self.set_latitude - self.latitude) 
		self.Iterm[1] = (self.Iterm[1] + self.errorX)*self.Ki[1]
		self.x_error = self.Kp[1]*self.errorX + self.Kd[1]*(self.errorX - self.prev_error[1]) +self.Iterm[1]
		self.prev_error[1] = self.errorX

                self.errorY = (self.set_longitude - self.longitude) 
		self.Iterm[2] = (self.Iterm[2] + self.errorY)*self.Ki[2]
		self.y_error = self.Kp[2]*self.errorY + self.Kd[2]*(self.errorY - self.prev_error[2]) +self.Iterm[2]
		self.prev_error[2] = self.errorY


                if self.errorZ > 0 and self.errorX > 0:
                        

		        print("**********")
		        print(self.errorZ)
		        print(self.z_error)
		        print("**********")

		        self.rc_pub.rcThrottle = 1500 + self.z_error

		
		        self.rc_pos_pub.publish(self.rc_pub)
                        


		        #self.z_pub.zerror = self.errorZ
		        #self.z_err_pub.publish(self.z_pub)




		if self.errorZ <= 0 and self.errorX > 0  :

			print("----------")
			print(self.errorX)
			print(self.x_error)
			print("----------")

			self.rc_pub.rcPitch = 1500 + self.x_error
                        self.rc_pub.rcRoll = 1500 + self.y_error
			self.rc_pub.rcThrottle = 1500 + self.z_error
			self.rc_pos_pub.publish(self.rc_pub)

			#self.x_pub.xerror = self.x_error
			#self.x_err_pub.publish(self.x_pub)

                if self.errorX <= 0  :
                        self.set_altitude = 0
                        self.errorZ = (self.set_altitude - self.altitude)
		        self.Iterm[0] = (self.Iterm[0] + self.errorZ)*self.Ki[0]
		        self.z_error = self.Kp[0]*self.errorZ + self.Kd[0]*(self.errorZ - self.prev_error[0]) +self.Iterm[0]
		        self.prev_error[0] = self.errorZ
		        print("**********")
		        print(self.errorZ)
		        print(self.z_error)
		        print("**********")

		        self.rc_pub.rcThrottle = 1500 + self.z_error
			self.rc_pos_pub.publish(self.rc_pub)
                        
               
                if self.errorZ == 3 and self.errorX <= 0:
                        self.rc_pub.rcRoll = 1500
                        self.rc_pub.rcPitch = 1500
                        self.rc_pub.rcYaw = 1500
                        self.rc_pub.rcThrottle = 1000
                      


                

		r.sleep()
		
		
		
		
		
if __name__ == '__main__':
    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
  
    while not rospy.is_shutdown():
        e_drone.pos_control()
        r.sleep()
