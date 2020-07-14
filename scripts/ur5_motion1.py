#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import time 
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
import numpy.matlib
from frompitoangle import *
from math import *
class null_space_control:
    def __init__(self):
        # moveit_commander.roscpp_initialize(sys.argv)
        # self.arm = moveit_commander.MoveGroupCommander('manipulator_ur5')
        rospy.init_node('moveit_fk_demo', anonymous=True)
        rospy.Subscriber('/joint_states', JointState, self.obtain_joint_states_sim, queue_size=1)

        self.robot = URDF.from_xml_file("/home/zy/catkin_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        self.aubo_q=[pi,-pi/2,pi/2,pi,-pi/2,0]
        self.aubo_q1=np.matrix(self.aubo_q)
        self.kp=50
        self.Kr=1
        self.r_dsr=np.matrix([-0.392,-0.109,0.609])
        self.rdot_dsr=np.matrix([0.0,0.0,0.0])
        self.rate=rospy.Rate(10) # 10hz
        self.urscript_pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
        self.error_r_pub=rospy.Publisher("/polishingrobot/error_r",Vector3,queue_size=10)
        self.t=0
        self.vel=1.05
        self.ace=1.4

    def obtain_joint_states_sim(self,msg):
        self.aubo_q=[]
        list1=msg.position
        for i in range(len(list1)):
            self.aubo_q.append(list1[i])
        self.aubo_q1=np.matrix(self.aubo_q)


    def kdl_computation(self):
        aubo_q=self.aubo_q
        kdl_kin = KDLKinematics(self.robot, "base_link", "wrist_3_link")
        pose = kdl_kin.forward(self.aubo_q) 
        J = kdl_kin.jacobian(self.aubo_q)
        return aubo_q, pose, J

    def qdot_generation1(self,aubo_qdes,aubo_qdotdes):
        aubo_q, pose, J=self.kdl_computation()
        J=J[0:3,0:6]

        tran_mat=pose[0:3,3]
        tran_mat=tran_mat.T

        rot_mat=pose[0:3,0:3]
        rot_mat=rot_mat.T

        zero_mat=np.matlib.zeros((3,3))
        mat1=np.hstack((rot_mat,zero_mat))
        mat2=np.hstack((zero_mat,rot_mat))
        base2endeffector_mat=np.vstack((mat1,mat2))
        inv_base2endeffector_mat=np.linalg.inv(base2endeffector_mat)

        pJ=np.dot(J.T,np.linalg.inv(np.dot(J,J.T)))
        null_mat=np.matlib.identity(6,dtype=float)-np.dot(pJ,J)
        delta_qdot=aubo_qdotdes-self.kp*(self.aubo_q1-aubo_qdes)
        
        rdot=tran_mat-self.r_dsr
        delta_rdot=self.rdot_dsr-self.Kr*rdot

        aubo_qdot=np.dot(pJ,delta_rdot.T)+np.dot(null_mat,delta_qdot.T)
        # aubo_qdot=np.dot(pJ,np.dot(inv_base2endeffector_mat[0:3,0:3],self.rdot_dsr.T))
        # aubo_qdot=np.dot(pJ,np.dot(inv_base2endeffector_mat[0:3,0:3],self.rdot_dsr.T))+np.dot(null_mat,delta_qdot.T)
        # aubo_qdot=np.dot(pJ,self.rdot_dsr.T)+np.dot(null_mat,delta_qdot.T)
        # aubo_qdot=np.dot(null_mat,delta_qdot.T)
        aubo_qdot=np.array([aubo_qdot[0,0],aubo_qdot[1,0],aubo_qdot[2,0],aubo_qdot[3,0],aubo_qdot[4,0],aubo_qdot[5,0]])
    
        mat=np.dot(J,null_mat)
        print("the mat is: ",mat)
        return aubo_qdot



    def moveur(self,q):
        ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(self.ace)+","+"v="+str(self.vel)+","+"t="+str(self.t)+")"
        # print ss
        self.urscript_pub.publish(ss)
        self.rate.sleep()
    
    def trans_error_pub(self):
        aubo_q, pose, J=self.kdl_computation()
        tran_mat=pose[0:3,3]
        tran_mat=tran_mat.T
        rdot=tran_mat-self.r_dsr
        error_r=Vector3()
        error_r.x=rdot[0,0]
        error_r.y=rdot[0,1]
        error_r.z=rdot[0,2]
        self.error_r_pub.publish(error_r)

    # def moveit_motion(self,joint_positions):    
    #     # 设置机械臂和夹爪的允许误差值
    #     self.arm.set_goal_joint_tolerance(0.01)
    #     self.arm.set_joint_value_target(joint_positions)
    #     self.arm.go()
    #     self.rate.sleep()

    # def moveit_shutdown(self):
    #     # 关闭并退出moveit
    #     moveit_commander.roscpp_shutdown()
    #     moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        step=0.05
        time1=100
        tnum=int(time1/step+1)
        omega=0.5
        radius=pi/4
        aubo=null_space_control()
        time.sleep(0.2)

        # aubo_q=[pi,-pi/2,pi/2,pi,-pi/2,0]
        # aubo.moveur(aubo_q)
        # time.sleep(0.5)
        # pose,J=aubo.kdl_computation()
        # print("pose is ", pose)

        for i in range(tnum):
        # for i in range(2):
            time1=time.time()

            t=step*i
            aubo_q,pose,J=aubo.kdl_computation()
            
            aubo_qdes=np.matrix([pi,-pi/2,pi/2+radius*sin(omega*t),pi,-pi/2,0.0])
            aubo_qdotdes=np.matrix([0.0,0.0,radius*omega*cos(omega*t),0.0,0.0,0.0])
            aubo_qdot=aubo.qdot_generation1(aubo_qdes,aubo_qdotdes)
            aubo_q=aubo_q+aubo_qdot*step
            for i in range(len(aubo_q)):
                if aubo_q[i]>355*pi/180.0:
                    aubo_q[i]=355*pi/180.0
                elif aubo_q[i]<-355*pi/180.0:
                    aubo_q[i]=-355/180.0*pi
            # print("aubo_q after controller is:",aubo_q)
            aubo.moveur(aubo_q)
            aubo.trans_error_pub()

            time2=time.time()
            # rospy.logerr("the last time is: {}".format(time2-time1))
            # step=time2-time1

        # aubo.moveit_shutdown()
    except rospy.ROSInterruptException:
        pass
