#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Range

balancing_point=10   #Center of the beam
error=0
rate_error=0
cum_error=0
kp=0.001
ki=0.001
kd=0.005
ball_radius=1.5

prev_error=0
prev_time=0

def callback(data):
 global prev_time
 global prev_error
 global cum_error

 rospy.loginfo(rospy.get_caller_id()+ "Distance is %s", data.range)

 #Calculating position
 current_pos= (data.range*100)+ball_radius
 current_pos= balancing_point-current_pos

 #Proportional error calculation
 error= current_pos

 #Rate error calculation
 cur_time= rospy.get_time()

 if prev_time==0:
  elapsed_time=0.2
 else:
  elapsed_time = cur_time-prev_time

 rate_error= (error-prev_error)/ elapsed_time

 #Cumulative error
 cum_error += error*elapsed_time

 #Changing value
 prev_time=cur_time
 prev_error= error

 value= (kp*error) + (kd*rate_error) + (ki*cum_error)

 rospy.loginfo("Publishing value: %s",value)
 print("Current position: ", current_pos)
 print("Elapsed time: ", elapsed_time)
 print("P term:", kp*error)
 print("I term:", ki*cum_error)
 print("D term:", kd*rate_error, "\n")

 pub.publish(value)

 
if __name__=='__main__':
 try:
  rospy.init_node("balancing_node",anonymous=True)
  pub=rospy.Publisher('/ball_balancer/seesaw_joint_position_controller/command', Float64, queue_size=10)
  rospy.Subscriber("sonar",Range,callback)
  rospy.spin()

 except rospy.ROSInterruptException:
  pass
