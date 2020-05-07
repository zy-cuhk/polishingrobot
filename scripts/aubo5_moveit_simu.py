#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import time 
from sensor_msgs.msg import JointState

class null_space_control:
    def __init__(self):
        # moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_fk_demo', anonymous=True)
        rospy.Subscriber('/joint_states', JointState, self.obtain_joint_states_sim, queue_size=1)

        # self.arm = moveit_commander.MoveGroupCommander('manipulator_i5')
        self.robot = URDF.from_xml_file("/home/zy/catkin_ws/src/aubo_robot-master/aubo_description/urdf/aubo_i5.urdf")
        self.aubo_q=[]
        self.aubo_qdes=[]
        self.aubo_qdotdes=[]
        self.null_mat=[]
        self.kp=1
        self.rdes=[0.0,0.0,0.0,0.0,0.0]

    def obtain_joint_states_sim(self,msg):
        list1=msg.position
        for i in range(len(list1)):
            self.aubo_q.append(list1[i])
        rospy.loginfo("aubo joints are: {}".format(self.aubo_q))


    def kdl_computation(self,q):
        # forward kinematics (returns homogeneous 4x4 matrix)
        kdl_kin = KDLKinematics(self.robot, "base_link", "wrist3_Link")
        pose = kdl_kin.forward(q) 
        # print pose
        J = kdl_kin.jacobian(q)
        # print 'J:', J
        return pose, J

    def qdot_generation(self):
        pJ=J'*inv(J*J')
        null_mat=eye(4)-pJ*J
        +null_mat*(self.aubo_qdotdes-self.kp*(self.aubo_q-self.aubo_qdes))

    def moveit_motion(self,joint_positions):    
        # 设置机械臂和夹爪的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go()

    def moveit_shutdown(self):
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        delta_t=0.1
        aubo=null_space_control()
        while(1):
            time1=time.time()

            joint_positions = [-1.0867, -1.274, 0.02832, 0.0820, -1.273, -0.003]
            aubo_qdot=aubo.qdot_generation()
            aubo_q=aubo_q+aubo_qdot*delta_t
            aubo.moveit_motion(aubo_q)

            time2=time.time()
            rospy.logerr("the last time is: {}".format(time2-time1))
            aubo.moveit_shutdown()
    except rospy.ROSInterruptException:
        pass
