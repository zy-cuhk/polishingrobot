#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import time 
from std_msgs.msg import String,Float64,Int64
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
        self.urscript_pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
        self.error_r_pub=rospy.Publisher("/polishingrobot/error_r",Vector3,queue_size=10)
        self.rdes_pub=rospy.Publisher("/polishingrobot/rdes",Vector3,queue_size=10)
        self.r_pub=rospy.Publisher("/polishingrobot/r",Vector3,queue_size=10)
        self.condition_num_pub=rospy.Publisher("/polishingrobot/condition_num",Float64,queue_size=10)
        self.rank_pub=rospy.Publisher("/polishingrobot/rank",Int64,queue_size=10)

        self.q4_pub=rospy.Publisher("/polishingrobot/q4",Float64,queue_size=10)
        self.q6_pub=rospy.Publisher("/polishingrobot/q6",Float64,queue_size=10)


        self.aubo_q=[1.50040841e-03, -2.83640237e+00, 1.53798406e+00, 1.29841831e+00, 1.50040840e-03, 3.14159265e+00]
        # self.aubo_q=[0.4169788306196942, -1.3621199297826534, -2.011464437502717, -2.22014083451496, -1.5707963267948966, 1.1538174961752024]
        self.aubo_q1=np.matrix(self.aubo_q)
        self.rate=rospy.Rate(10) # 10hz
        self.vel=1.05
        self.ace=1.4
        self.t=0

        self.kpr=10
        self.kq4=1000
        self.kq6=1000        
        self.bq4=pi/20
        self.bq6=pi/20

    def obtain_joint_states_sim(self,msg):
        pass
        self.aubo_q=[]
        list1=msg.position
        for i in range(len(list1)):
            self.aubo_q.append(list1[i])
        self.aubo_q1=np.matrix(self.aubo_q)


    def kdl_computation(self):
        aubo_q=self.aubo_q
        kdl_kin = KDLKinematics(self.robot, "base_link", "ee_link")
        pose = kdl_kin.forward(self.aubo_q) 
        J = kdl_kin.jacobian(self.aubo_q)
        return aubo_q, pose, J


    # def qdot_generation(self,aubo_rdes,aubo_rdot_des):
    #     aubo_q, pose, J1=self.kdl_computation()
    #     J2_1=np.matrix([[J1[0,1],J1[0,2],J1[0,3],J1[0,5]]])
    #     J2_2=np.matrix([[J1[2,1],J1[2,2],J1[2,3],J1[2,5]]])
    #     J=np.vstack((J2_1,J2_2))
    #     print("J is:",J)

    #     position_x=pose[0,3]
    #     position_y=pose[1,3]
    #     position_z=pose[2,3]
    #     tran_mat=np.matrix([[position_x],[position_y],[position_z]])
    #     delta_rdot=aubo_rdot_des-self.kpr*(tran_mat-aubo_rdes)
    #     print("aubo_rdot_des is", aubo_rdot_des)
    #     print("tran_mat is",tran_mat)
    #     print("aubo_rdes is",aubo_rdes)
    #     print("delta_rdot is", delta_rdot)

    #     print("the combined J is:",np.dot(J,J.T))
    #     pJ=np.dot(J.T,np.linalg.inv(np.dot(J,J.T)))
    #     print("pJ is",pJ)
    #     null_mat=np.matlib.identity(4,dtype=float)-np.dot(pJ,J)

    #     fq4=aubo_q[3]**2-self.bq4**2
    #     print("fq4 is:",fq4)
    #     q4=self.kq4*min(0,fq4)
    #     print("q4 is:",q4)
    #     fq6=aubo_q[5]**2-self.bq6**2
    #     print("fq6 is:",fq6)
    #     q6=self.kq6*min(0,fq6)
    #     print("q6 is:",q6)
    #     delta_qdot=np.matrix([0.0,0.0,q4,q6])

    #     delta_rdot1=np.matrix([[delta_rdot[0,0]],[delta_rdot[2,0]]])
    #     aubo_qdot1=np.dot(pJ,delta_rdot1)      
    #     # aubo_qdot1=np.dot(pJ,delta_rdot)+np.dot(null_mat,delta_qdot.T)
    #     aubo_qdot=np.array([0.0,aubo_qdot1[0,0],aubo_qdot1[1,0],aubo_qdot1[2,0],0.0,aubo_qdot1[3,0]])
    #     print("aubo_qdot is:",aubo_qdot)
    #     mat=np.dot(J,null_mat)
    #     print("the mat is: ",mat)

    #     condition_num=np.linalg.det(np.dot(J,J.T))
    #     self.condition_num_pub.publish(condition_num)
    #     return aubo_qdot


    def qdot_generation1(self,aubo_rdes,aubo_rdot_des):
        # aubo_q, pose, J1=self.kdl_computation()
        while(1):
            aubo_q, pose, J1=self.kdl_computation()
            if not (pose is None):
                break
            time.sleep(0.01)
            
        J2_1=np.matrix([[J1[0,1],J1[0,2],J1[0,3],J1[0,5]]])
        J2_2=np.matrix([[J1[2,1],J1[2,2],J1[2,3],J1[2,5]]])
        J=np.vstack((J2_1,J2_2))
        print("J is:",J)

        position_x=pose[0,3]
        position_y=pose[1,3]
        position_z=pose[2,3]
        tran_mat=np.matrix([[position_x],[position_y],[position_z]])
        delta_rdot=aubo_rdot_des-self.kpr*(tran_mat-aubo_rdes)
        print("aubo_rdot_des is", aubo_rdot_des)
        print("tran_mat is",tran_mat)
        print("aubo_rdes is",aubo_rdes)
        print("delta_rdot is", delta_rdot)

        print("the combined J is:",np.dot(J,J.T))
        pJ=np.dot(J.T,np.linalg.pinv(np.dot(J,J.T)))
        print("pJ is",pJ)
        null_mat=np.matlib.identity(4,dtype=float)-np.dot(pJ,J)

        fq4=aubo_q[2]**2-self.bq4**2
        print("fq4 is:",fq4)
        q4=self.kq4*min(0,fq4)
        print("q4 is:",q4)
        fq6=aubo_q[5]**2-self.bq6**2
        print("fq6 is:",fq6)
        q6=self.kq6*min(0,fq6)
        print("q6 is:",q6)
        self.q4_pub.publish(q4)
        self.q6_pub.publish(q6)

        delta_qdot=np.matrix([0.0,q4,0.0,q6])

        delta_rdot1=np.matrix([[delta_rdot[0,0]],[delta_rdot[2,0]]])
        # aubo_qdot1=np.dot(pJ,delta_rdot1)      
        aubo_qdot1=np.dot(pJ,delta_rdot1)+np.dot(null_mat,delta_qdot.T)
        aubo_qdot=np.array([0.0,aubo_qdot1[0,0],aubo_qdot1[1,0],aubo_qdot1[2,0],0.0,aubo_qdot1[3,0]])
        print("aubo_qdot is:",aubo_qdot)
        mat=np.dot(J,null_mat)
        print("the mat is: ",mat)

        condition_num=np.linalg.det(np.dot(J,J.T))
        self.condition_num_pub.publish(condition_num)

        tran_mat=pose[0:3,3]
        rdot=tran_mat-aubo_rdes

        error_r=Vector3()
        error_r.x=rdot[0,0]
        error_r.y=rdot[1,0]
        error_r.z=rdot[2,0]
        rdes=Vector3()
        rdes.x=aubo_rdes[0,0]
        rdes.y=aubo_rdes[1,0]
        rdes.z=aubo_rdes[2,0]
        r=Vector3()
        r.x=tran_mat[0,0]
        r.y=tran_mat[1,0]
        r.z=tran_mat[2,0] 

        self.error_r_pub.publish(error_r)
        self.rdes_pub.publish(rdes)
        self.r_pub.publish(r)

        return aubo_qdot

    def moveur(self,q):
        ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(self.ace)+","+"v="+str(self.vel)+","+"t="+str(self.t)+")"
        self.urscript_pub.publish(ss)
        self.rate.sleep()
    
    # def trans_error_pub(self,aubo_rdes):
    #     aubo_q, pose, J=self.kdl_computation()
    #     tran_mat=pose[0:3,3]
    #     rdot=tran_mat-aubo_rdes
    #     error_r=Vector3()
    #     error_r.x=rdot[0,0]
    #     error_r.y=rdot[1,0]
    #     error_r.z=rdot[2,0]
    #     rdes=Vector3()
    #     rdes.x=aubo_rdes[0,0]
    #     rdes.y=aubo_rdes[1,0]
    #     rdes.z=aubo_rdes[2,0]
    #     r=Vector3()
    #     r.x=tran_mat[0,0]
    #     r.y=tran_mat[1,0]
    #     r.z=tran_mat[2,0]        

    #     self.error_r_pub.publish(error_r)
    #     self.rdes_pub.publish(rdes)
    #     self.r_pub.publish(r)
        # self.rate.sleep()

    # def rdes_pub(self,aubo_rdes):
    #     rdes=Vector3()
    #     rdes.x=aubo_rdes[0,0]
    #     rdes.y=aubo_rdes[1,0]
    #     rdes.z=aubo_rdes[2,0]
    #     self.rdes_pub.publish(rdes)

    # def rank_pub1(self):
    #     aubo_q, pose, J=self.kdl_computation()
    #     rank=np.linalg.matrix_rank(J)
    #     self.rank_pub.publish(rank)
    #     # self.rate.sleep()

    # def condition_num_pub1(self):
    #     aubo_q, pose, J=self.kdl_computation()
    #     condition_num=np.linalg.det(np.dot(J,J.T))
    #     print("condition_num",condition_num)
    #     self.condition_num_pub.publish(condition_num)
    #     self.rate.sleep()

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
        step=0.01
        time1=10
        tnum=int(time1/step+1)
        omega=0.5
        radius=0.15

        flag=1

        aubo=null_space_control()
        if flag==0:
            # aubo_q=[1.50040841e-03, -2.83640237e+00, 1.53798406e+00, 1.29841831e+00, 1.50040840e-03, 3.14159265e+00]
            aubo_q=[0.0, -162.47, 38.60, 74.39, 0.0, 180.0]
            for i in range(len(aubo_q)):
                aubo_q[i]=aubo_q[i]/180*pi
            for i in range(100):
                aubo.moveur(aubo_q)

        time.sleep(5)
        aubo_q, pose, J1=aubo.kdl_computation()
        print("aubo_q is:",aubo_q)
        print("pose is: ",pose)
        position_x=pose[0,3]
        position_z=pose[2,3]
        print("position_x is:",position_x)
        print("position_z is:",position_z)
        print("sin is:",sin(pi/2))

        if flag==1:
            for i in range(tnum):
                time1=time.time()
                t=step*i

                aubo_rdes=np.matrix([radius*cos(omega*t)+position_x-radius, 0.191, radius*sin(omega*t)+position_z])
                aubo_rdes=aubo_rdes.T
                aubo_rdot_des=np.matrix([-radius*omega*sin(omega*t), 0.0, radius*omega*cos(omega*t)])
                aubo_rdot_des=aubo_rdot_des.T
                aubo_qdot=aubo.qdot_generation1(aubo_rdes,aubo_rdot_des)

                aubo_q=aubo_q+aubo_qdot*step
                aubo_q[0]=0.0
                aubo_q[4]=0.0
                for i in range(len(aubo_q)):
                    if aubo_q[i]>355*pi/180.0:
                        aubo_q[i]=355*pi/180.0
                    elif aubo_q[i]<-355*pi/180.0:
                        aubo_q[i]=-355/180.0*pi
                print("aubo_q is:",aubo_q)
                aubo.moveur(aubo_q)
                


            # aubo_rdes1=np.matrix([radius*cos(omega*t)+position_x, 0.192, radius*sin(omega*t)+position_z])
            # aubo_rdes1=aubo_rdes1.T
            # aubo.trans_error_pub(aubo_rdes1)

        #     time2=time.time()
        #     rospy.logerr("the last time is: {}".format(time2-time1))
        #     step=time2-time1

        # aubo.moveit_shutdown()
    except rospy.ROSInterruptException:
        pass
