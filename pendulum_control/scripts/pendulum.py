#!/usr/bin/python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates

rospy.init_node("pendulum")
p = rospy.Publisher("/turtle1/cmd_vel", Twist)
debug_angle = rospy.Publisher("/info/angle", Float32)
debug_speed = rospy.Publisher("/info/speed", Float32)
debug_value = rospy.Publisher("/info/value", Float32)
debug_err   = rospy.Publisher("/info/err", Float32)

def angle_diff(a1, a2):
    a = a1-a2
    return (a+math.pi)%(2*math.pi)-math.pi

old_pos = 0
old_velocity = 0
vel_counter = 0

def swing(msg):
  error_sign = error/abs(error)
  return -1.5*error_sign#velocity_sign

error_sum = 0
error_array = []
position = 0
last_error = None

def reset():
  global error_sum, last_error, error_array
  last_error = None
  error_sum = 0  
  error_array = []


def balance(error):
  global error_sum, last_error, error_array
  error_array = [error] + error_array
  if len(error_array) > 50:
     error_array.pop()     
  error_sum = sum(error_array)

  if last_error == None:
    last_error = error
    error_diff = 0
  else:
    error_diff = error-last_error
    last_error = error
  rospy.logwarn("p%s,i%s,d%s,le%s",error,error_sum,error_diff, last_error)
  return -30*error - 1*error_sum - 0*error_diff

def callback(msg):
  angle = msg.position[0] % (math.pi*2)

  error = angle_diff(0, angle)
  error_sign = error/abs(error)
  is_balancing = abs(error) < math.pi/3
  if abs(position) < 5 or is_balancing:
    if is_balancing:
      global position
      result = balance(error - position*0.05)
    else:
      reset()
      result = 1.5*error_sign #swing
  else:
    reset()
    result = -5*position/abs(position) #return robot to center 

  value = Twist()
  value.linear.x = result
  p.publish(value)

  debug_value.publish(Float32(result))
  debug_err.publish(error)
  speed = msg.velocity[0]
  debug_angle.publish(Float32(angle))
  debug_speed.publish(Float32(speed))


def model_state(msg):
  global position
  position = msg.pose[1].position.x
 
s = rospy.Subscriber("/rrbot/joint_states", JointState, callback)
s = rospy.Subscriber("/gazebo/model_states", ModelStates, model_state)

def myhook():
  p.publish(Twist())

rospy.on_shutdown(myhook)

while not rospy.is_shutdown():
  rospy.spin()

