#!/usr/bin/python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
import numpy as np
import scipy.linalg

MAX_FORCE = 10


def solve_discrete_are(a, b, q, r):
    g = np.dot(np.dot(b, g), b.conj().transpose())

    try:
        ait = inv(a).conj().transpose()  # ait is "A inverse transpose"
    except LinAlgError:
        raise ValueError('Matrix A in the algebraic Riccati equation solver is ill-conditioned')

    z11 = a+np.dot(np.dot(g, ait), q)
    z12 = -1.0*np.dot(g, ait)
    z21 = -1.0*np.dot(ait, q)
    z22 = ait

    z = np.vstack((np.hstack((z11, z12)), np.hstack((z21, z22))))

    # Note: we need to sort the upper left of s to lie within the unit circle,
    #       while the lower right is outside (Laub, p. 7)
    [s, u, sorted] = schur(z, sort='iuc')

    (m,n) = u.shape

    u11 = u[0:m//2, 0:n//2]
    u21 = u[m//2:m, 0:n//2]
    u11i = inv(u11)

    return np.dot(u21, u11i)


def get_k(M, m, I, g, l):
    b = 0.0
    A = np.array([
        [0, 1, 0, 0],
        [0, -(I+m*l*l)*b/p, m*m*g*l*l/p, 0],
        [0, 0, 0, 1],
        [0, -(m*l*b)/p, m*g*l*(M+m)/p, 0]
        ], np.float)
    
    B = np.array([[0], 
                 [(I+m*l*l)/p], 
                 [0],
                 [m*l/p]])
    C = np.array([[1, 0, 0, 0],
                 [0, 0, 1, 0]], np.float)
    D = np.array([[0],
                  [0]])
    
    Q =np.dot(C.T, C) 
    R = np.array([[1]])
    
    return lqr(A, B, Q, R)


def lqr(A,B,Q,R):
    """Solve the continuous time lqr controller.
     
    dx/dt = A x + B u
     
    cost = integral x.T*Q*x + u.T*R*u
    """
    #ref Bertsekas, p.151
 
    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
     
    #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
     
    # eigVals, eigVecs = scipy.linalg.eig(A-B*K)
     
    return K  # , X, eigVals

def get_k(M, m, I, g, l):
    b = 0.0
    p = I*(M*m)+M*m*l**2
    A = np.array([
        [0, 1, 0, 0],
        [0, -(I+m*l*l)*b/p, m*m*g*l*l/p, 0],
        [0, 0, 0, 1],
        [0, -(m*l*b)/p, m*g*l*(M+m)/p, 0]
        ], np.float)
    
    B = np.array([[0], 
                 [(I+m*l*l)/p], 
                 [0],
                 [m*l/p]])
    C = np.array([[1, 0, 0, 0],
                 [0, 0, 1, 0]], np.float)
    D = np.array([[0],
                  [0]])
    
    Q =np.dot(C.T, C) 
    Q[0,0] = 10 #5000
    Q[2,2] = 100
    R = np.array([[1]])  # 1500
    return lqr(A, B, Q, R)


class Logger:
    def __init__(self, topic_name):
        self.publisher = rospy.Publisher("/log/" + topic_name, Float32, queue_size=10)

    def publish(self, data):
        self.publisher.publish(Float32(data))


def angle_diff(a1, a2):
    a = a1-a2
    return (a+math.pi)%(2*math.pi)-math.pi


class PendulumController:
    def __init__(self):
        rospy.init_node("pendulum")

        self.log_angle = Logger("angle")
        self.log_force = Logger("force")
        self.log_angle_speed = Logger("angle_speed")
        self.log_position = Logger("position")
        self.log_caret_speed = Logger("caret_speed")

        self.log_i_part = Logger("i_part")
        self.log_d_part = Logger("d_part")

        rospy.wait_for_service('/gazebo/unpause_physics')

        self.publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/rrbot/joint_states", JointState, self.process)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state)
        self.angle = 0
        self.position = 0
        self.prev_angle_time = rospy.Time.now()
        self.prev_position_time = rospy.Time.now()
        self.integral_error = 0
        self.prev_error = 0

        self.k_p = 27
        self.k_i = 0
        self.k_d = 4.15

        M = 0.5
        m = 0.2
        I = 0.006
        g = 9.8
        l = 0.3
        p=0.0132

        self.lqr_k = get_k(M,m,I,g,l)
        self.force = 0

        self.f = open("/home/mik/pendulum.csv","w")
        self.f.write("time, car position,Car speed,angle,angle_speed,force\r\n")
        #f.close()

    def model_state(self, msg):
        stamp = rospy.Time.now()
        self.position_time_delta = stamp - self.prev_position_time
        #rospy.logwarn("X: %s", self.position_time_delta.to_sec())
        self.prev_position_time = stamp
        self.prev_position = self.position
        if len(msg.pose) >= 1:
            self.position = msg.pose[1].position.x
            if (self.position_time_delta.to_sec() == 0):
                return
            self.speed = (self.position - self.prev_position)/self.position_time_delta.to_sec()


    def get_data(self, msg):
        stamp = rospy.Time.now()
        self.angle_time_delta = stamp - self.prev_angle_time
        # rospy.logwarn("Angle: %s", self.angle_time_delta.to_sec())
        self.prev_angle_time = stamp
        self.prev_angle = self.angle 
        self.angle = msg.position[0] % (math.pi*2)
        self.angle_error = angle_diff(0, self.angle)
        if (self.angle_time_delta.to_nsec() == 0):
            return
        else:
            self.angle_speed = (self.angle - self.prev_angle)/self.angle_time_delta.to_sec()

    def log(self):
        #f = open("/home/mik/pendulumicsv","w+")
        self.f.write("%s,%s,%s,%s,%s,%s\r\n"%(rospy.Time.now(), self.position, self.speed, self.angle, self.angle_speed, self.force))
        #f.close()
        #self.log_angle.publish(self.angle)
        #self.log_force.publish(self.force)
        #self.log_angle_speed.publish(self.angle_speed)
        #self.log_position.publish(self.position)
        #self.log_caret_speed.publish(self.speed)
        #self.log_i_part.publish(self.integral_error) 
        #self.log_d_part.publish(self.diff_error) 


    def pid_regulator(self):
        angle_error = self.angle_error - 0.1*self.position
        self.integral_error += angle_error
        self.integral_error = max(-100, min(100, self.integral_error))
        self.diff_error = angle_error - self.prev_error
        self.prev_error = angle_error
        force = self.k_p*self.angle_error + self.k_i*self.integral_error + self.k_d*self.diff_error
        return -force

    def lqr_regulator(self):
        self.diff_error = self.angle_error - self.prev_error
        self.prev_error = self.angle_error
        force=-self.lqr_k[0,0]*self.position-self.lqr_k[0,1]*self.speed-self.lqr_k[0,2]*self.angle_error-self.lqr_k[0,3]*self.diff_error
        return force


    def process(self, msg):
        self.get_data(msg) 
        value = Twist()

        self.force = self.pid_regulator()
        #self.force = self.lqr_regulator()
        self.force = min(MAX_FORCE, max(-MAX_FORCE, self.force))

        value.linear.x = self.force  
        self.publisher.publish(value)
        self.log()

controller = PendulumController()

while not rospy.is_shutdown():
    pass
