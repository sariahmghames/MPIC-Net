import numpy as np
import os
import numpy as np
import rospy
from PIL import Image
import os.path
import random
import math
import bpy
from bpy.types import Operator
import mathutils
 
import logging
import time
import smach # creates hierarchical state machine (can be used to trigger the simulation after simulation time runs out)
import smach_ros
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Imu

# 3D Cluster Generation
# N is the nb of fruits in the cluster (in one view)
# R is the fruit diameter
# S is the cluster size

class Motion_Gen:

    def __init__(self,):
    	rospy.init_node('Data Collection', anonymous = True)
    	rospy.rate = 50
        self.N = 10
        self.R = 3
		self.S = 15
		self.cluster_ID = 0
		self.q = []
        self.qdot = []
        self.stiff = []
        self.time = np.linspace(0,1,100, endpoint = True)
        self.palm_wrist = 0.05
        self.wrist_elb = 0.2
        self.elb_shld = 0.2
        self.kl_min = 100 # N/m
        self.kl_max = 600 # N/m
        self.kw_min = 10 # N/m
        self.kw_max = 60 # N/m
        self.T = 1
        self.dt = self.T/100
        self.key_start = []
        self.imu0_quat_sub = rospy.Subscriber('/imu', Imu, uparm_imu0_callbk_quat)
        self.imu1_quat_sub = rospy.Subscriber('/imu', Imu, forearm_imu1_callbk_quat)
        self.imu2_quat_sub = rospy.Subscriber('/imu', Imu, wrist_imu2_callbk_quat)
        self.imu0_euler_sub = rospy.Subscriber('/imu_euler', Float64MultiArray, uparm_imu0_callbk_eul)
        self.imu1_euler_sub = rospy.Subscriber('/imu_euler', Float64MultiArray, forearm_imu1_callbk_eul)
        self.imu2_euler_sub = rospy.Subscriber('/imu_euler', Float64MultiArray, wrist_imu2_callbk_eul)
        self.myo_sub = rospy.Subscriber('/myo', , myo_callbk) 
        self.Kee = np.zeros(6,6)
        self.Dee = np.zeros(6,6)
        self.lu_u = np.transpose([self.elb_shld, 0, 0])
        self.lf_f = np.transpose([self.wrist_elb, 0, 0])
        self.lh_h = np.transpose([self.palm_wrist, 0, 0])
        self.subJState  = rospy.Subscriber("/joint_states", JointState, callback=self.RobotStateCallback)
        self.q_current = []
        self.qdot_current = []


   def RobotStateCallback(self,msg):
        name = msg.name
        print('msg state=', msg)
        self.q_current = msg.position[2:]
        print('q0=', self.q_current)
        self.qdot_current = msg.velocity[2:]



    def load_clusters(self,):

        print('wait until world is loaded to start picking demo')
        # create a listener here to handle a message that world was loaded
        print('you can start picking press the key before to trigger registration')
        while self.key_start isempty():
            self.key_start = np.input()
        arm_data()


	def arm_data(self,):
        H_qs_roll_d, H_qs_pitch, H_qs_yaw = uparm_imu0_callbk_eul()
        H_qe_roll, H_qe_pitch, H_qe_yaw = forearm_myo_callbk()
        H_qw_roll, H_qw_pitch, H_qw_yaw = wrist_imu2_callbk_eul()
        s_act = uparm_emg()
        f_act = forearm_emg()
        w_act = wirst_emg()
        kl = kl_min + (kl_max - kl_min) * f_act
        kw = kw_min + (kw_max - kw_min) * f_act

        self.Kee[0:3, 0:3] = kl*np.eye(3,3)
        self.Kee[3:6, 3:6] = kw*np.eye(3,3)
        self.Dee = 0.5 * self.Kee
        Rh_hand_world = Rh_sh_world * Rh_forearm_sh * Rh_hand_forearm
        th_hand_world = ts_0 + Ru_0 * self.tu_u + Rf_0 * self.tf_f + Rh_0 * self.lh_h
        Th_hand_world = tf_tran.concatenate_matrices(Rh_hand_world, th_hand_world)

        #self.robot_imitation(H_qs_roll_d, H_qs_pitch_d, H_qs_yaw_d, H_qe_roll_d, H_qe_pitch_d, H_qe_yaw_d, H_qw_roll_d, H_qw_pitch_d, H_qw_yaw_d)
        self.robot_imitation(Th_hand_world)



    def robot_imitation(self, T_desired):

        tee_d = T_desired[0:3,-1]  # x translation
        euler_d = tf_tran.matrix_euler(T_desired[0:3,0:3])  # x rotation
        Tee_d = T_desired
        Xd = [tee_d[0], tee_d[1], tee_d[2], euler_d[0], euler_d[1], euler_d[2]]
        dXd = np.diff(Xd)
        while len(self.q_current == 0):
            continue
        qR_d =  franka_kin.inv_kin2(self.q_current, T_desired)
        qdotR_d = np.diff(qR_d)  
        g = 0 # if embedded in the controller you want to use in the joint space
        Tj, Tee_current = franka_kin.fwd_kin(qR_d)
        Jee = franka_kin.geometric_jacobian(Tj, Tee_current)
        Kj_d = Jee.T.dot(self.Kee)
        Dj_d = Jee.T.dot(self.Dee)
        tau_R_d = Jee.T.dot(self.Kee * Xd + self.Dee*dXd) + g

        return tauj, qR_d, qdotR_d, Kj_d, Dj_d


    def arm_IK(self,qs_roll, qs_pitch, qs_yaw, qe_roll, qe_pitch, qe_yaw, qw_roll, qw_pitch, qw_yaw, arm_stiff_operat):
        X_palm, euler_palm = arm_fwdkin(qs_roll, qs_pitch, qs_yaw, qe_roll, qe_pitch, qe_yaw, qw_roll, qw_pitch, qw_yaw)
        euler_EE = euler_palm
        Xee = X_palm
        q_franka = franka_IK(Xee, euler_EE)
        qdot_franka = np.concatenate(np.diff(q_franka)/self.dt, np.zeros(1)) 
        EE_stiff = np.ones(6)*arm_stiff_operat
        EE_stiff_mat = np.diag(EE_stiff)
        Jacob = franka_Jacob()
        franka_stiff_joint = Jacob.transpose().dot(EE_stiff_mat)
        self.q.append(q_franka)
        self.qdot.append(qdot_franka)
        self.stiff.append(franka_stiff_joint)



    def arm_fwdkin(self,):

    def franka_Jacob(self,):

    def franka_IK(self,):


    def uparm_imu0_callbk_eul(self,msg):
    	return None

    def forearm_imu1_callbk_eul(self,msg):
    	return None


    def wrist_imu2_callbk_eul(self,msg):
    	self.wrist_roll = msg.data[0]
    	self.wrist_pitch = msg.data[1]
    	self.wrist_yaw = msg.data[2]

    
    def forearm_myo_callbk(self,msg):



    def save_data(self,):
        save(self.q, self.qdot,self.stiff)
				
if __name__ == '__main__':
    load_clusters()


