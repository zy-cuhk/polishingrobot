#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import time 
from sensor_msgs.msg import JointState

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
import numpy.matlib
from frompitoangle import *
from math import *
class null_space_control:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_fk_demo', anonymous=True)
        rospy.Subscriber('/joint_states', JointState, self.obtain_joint_states_sim, queue_size=1)

        self.arm = moveit_commander.MoveGroupCommander('manipulator_i5')
        self.robot = URDF.from_xml_file("/home/zy/catkin_ws/src/aubo_robot-master/aubo_description/urdf/aubo_i5.urdf")
        self.aubo_q=[0,pi/2,pi/2,pi/2,0,0]
        self.aubo_q1=np.matrix(self.aubo_q)
        self.kp=50
        self.rdot_dsr=np.matrix([0.0,0.0,0.0])
        self.rate=rospy.Rate(10) # 10hz

    def obtain_joint_states_sim(self,msg):
        self.aubo_q=[]
        list1=msg.position
        for i in range(len(list1)):
            self.aubo_q.append(list1[i])
        if len(self.aubo_q)!=0:
            pass
        else:
            self.aubo_q=[0,pi/2,pi/2,pi/2,0,0]
        self.aubo_q1=np.matrix(self.aubo_q)
        rospy.loginfo("aubo joints are: {}".format(self.aubo_q))


    def kdl_computation(self):
        # forward kinematics (returns homogeneous 4x4 matrix)
        kdl_kin = KDLKinematics(self.robot, "base_link", "wrist3_Link")
        # self.aubo_q=np.asarray(self.aubo_q)
        pose = kdl_kin.forward(self.aubo_q) 
        # print pose
        J = kdl_kin.jacobian(self.aubo_q)
        # print 'J:', J

        return pose, J

    def qdot_generation(self,aubo_qdes,aubo_qdotdes):
        # forward kinematics (returns homogeneous 4x4 matrix)
        kdl_kin = KDLKinematics(self.robot, "base_link", "wrist3_Link")
        # self.aubo_q=np.asarray(self.aubo_q)
        pose = kdl_kin.forward(self.aubo_q) 
        # print pose
        J = kdl_kin.jacobian(self.aubo_q)
        # print 'J:', J
        # print(type(pose))
        # print(type(J))
        aubo_q1=np.matrix(self.aubo_q)
        J=J[0:3,0:6]
        # print("new_j is",J)
        rot_mat=pose[0:3,0:3]
        # print("rot_mat is",rot_mat)
        rot_mat=rot_mat.T
        # print("inverse rot_mat is",rot_mat)
        zero_mat=np.matlib.zeros((3,3))
        mat1=np.hstack((rot_mat,zero_mat))
        mat2=np.hstack((zero_mat,rot_mat))
        base2endeffector_mat=np.vstack((mat1,mat2))
        # print("base2endeffector_mat is",base2endeffector_mat)
        inv_base2endeffector_mat=np.linalg.inv(base2endeffector_mat)
        # print("inv_base2endeffector_mat is",inv_base2endeffector_mat[0:5,0:5])
        pJ=np.dot(J.T,np.linalg.inv(np.dot(J,J.T)))
        # print("pJ is:",pJ)
        null_mat=np.matlib.identity(6,dtype=float)-np.dot(pJ,J)
        # print("null_mat is",null_mat)
        delta_qdot=aubo_qdotdes-self.kp*(aubo_q1-aubo_qdes)
        # print("delta_qdot",delta_qdot)
        aubo_qdot=np.dot(pJ,np.dot(inv_base2endeffector_mat[0:3,0:3],self.rdot_dsr.T))+np.dot(null_mat,delta_qdot.T)
        # aubo_qdot=np.dot(pJ,self.rdot_dsr.T)+np.dot(null_mat,delta_qdot.T)
        # aubo_qdot=np.dot(null_mat,delta_qdot.T)
        aubo_qdot=np.array([aubo_qdot[0,0],aubo_qdot[1,0],aubo_qdot[2,0],aubo_qdot[3,0],aubo_qdot[4,0],aubo_qdot[5,0]])
        print("aubo_qdot is: ",aubo_qdot)
        return aubo_qdot

    def qdot_generation1(self,aubo_q,aubo_qdes,aubo_qdotdes):
        # forward kinematics (returns homogeneous 4x4 matrix)
        kdl_kin = KDLKinematics(self.robot, "base_link", "wrist3_Link")
        # self.aubo_q=np.asarray(self.aubo_q)
        pose = kdl_kin.forward(aubo_q) 
        # print pose
        J = kdl_kin.jacobian(aubo_q)
        # print 'J:', J
        # print(type(pose))
        # print(type(J))
        aubo_q1=np.matrix(aubo_q)
        J=J[0:3,0:6]
        # print("new_j is",J)
        rot_mat=pose[0:3,0:3]
        # print("rot_mat is",rot_mat)
        rot_mat=rot_mat.T
        # print("inverse rot_mat is",rot_mat)
        zero_mat=np.matlib.zeros((3,3))
        mat1=np.hstack((rot_mat,zero_mat))
        mat2=np.hstack((zero_mat,rot_mat))
        base2endeffector_mat=np.vstack((mat1,mat2))
        # print("base2endeffector_mat is",base2endeffector_mat)
        inv_base2endeffector_mat=np.linalg.inv(base2endeffector_mat)
        # print("inv_base2endeffector_mat is",inv_base2endeffector_mat[0:5,0:5])
        pJ=np.dot(J.T,np.linalg.inv(np.dot(J,J.T)))
        # print("pJ is:",pJ)
        null_mat=np.matlib.identity(6,dtype=float)-np.dot(pJ,J)
        # print("null_mat is",null_mat)
        delta_qdot=aubo_qdotdes-self.kp*(aubo_q1-aubo_qdes)
        # print("delta_qdot",delta_qdot)
        aubo_qdot=np.dot(pJ,np.dot(inv_base2endeffector_mat[0:3,0:3],self.rdot_dsr.T))+np.dot(null_mat,delta_qdot.T)
        # aubo_qdot=np.dot(pJ,self.rdot_dsr.T)+np.dot(null_mat,delta_qdot.T)
        # aubo_qdot=np.dot(null_mat,delta_qdot.T)
        aubo_qdot=np.array([aubo_qdot[0,0],aubo_qdot[1,0],aubo_qdot[2,0],aubo_qdot[3,0],aubo_qdot[4,0],aubo_qdot[5,0]])
        print("aubo_qdot is: ",aubo_qdot)
        return aubo_qdot

    def moveit_motion(self,joint_positions):    
        # 设置机械臂和夹爪的允许误差值
        self.arm.set_goal_joint_tolerance(0.01)
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go()
        self.rate.sleep()

    def moveit_shutdown(self):
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        step=0.01
        time1=20
        tnum=int(time1/step+1)
        omega=5
        radius=pi/4
        aubo=null_space_control()
        time.sleep(0.2)
        aubo_q=[0,pi/2,pi/2,pi/2,0,0]
        aubo.moveit_motion(aubo_q)
        for i in range(tnum):
        # for i in range(1):
            time1=time.time()

            t=step*(i-1)
            aubo_qdes=np.matrix([0.0,pi/2,pi/2+radius*sin(omega*t),pi/2,0.0,0.0])
            aubo_qdotdes=np.matrix([0.0,0.0,radius*omega*cos(omega*t),0.0,0.0,0.0])
            # aubo_qdot=aubo.qdot_generation(aubo_qdes,aubo_qdotdes)
            aubo_qdot=aubo.qdot_generation1(aubo_q,aubo_qdes,aubo_qdotdes)
            aubo_q=aubo_q+aubo_qdot*step
            # print("aubo_q before is:",aubo_q)
            # for i in range(len(aubo_q)):
            #     if aubo_q[i]>170.0*pi/180.0:
            #         aubo_q[i]=170.0*pi/180.0
            #     elif aubo_q[i]<-170.0*pi/180.0:
            #         aubo_q[i]=-170.0/180.0*pi
            print("aubo_q is:",aubo_q)
            aubo.moveit_motion(aubo_q)

            time.sleep(1)
            time2=time.time()
            rospy.logerr("the last time is: {}".format(time2-time1))
            # step=time2-time1
        aubo.moveit_shutdown()
    except rospy.ROSInterruptException:
        pass
# rosrun tf tf_echo /base_link /wrist3_Link

