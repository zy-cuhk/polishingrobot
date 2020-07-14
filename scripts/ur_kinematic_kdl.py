#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Import the module
import sys
# print sys.path
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
from numpy.matlib import *

# robot = URDF.from_xml_file("../urdf/test_robot.urdf")
# robot = URDF.from_xml_file("../urdf/ur3.urdf")

class UR_robot:
    def __init__(self):
        # rospy.init_node("import_ur3_from_urdf")
        self.robot = self.init_robot("/home/zy/catkin_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        self.kdl_kin = KDLKinematics(self.robot, "base_link", "ee_link")
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain("base_link", "ee_link")
        # safe angle of UR3

        self.safe_q = [1.3189744444444444, -2.018671111111111, 1.8759755555555557, 2.7850055555555557, 0.17444444444444443, 3.7653833333333337]
        self.q = self.safe_q

    def init_robot(self, filename):
        # print("here")
        robot = URDF.from_xml_file(filename)
        # print("outhere")
        return robot

    def set_q(self, q_list):
        self.q = q_list

    def get_fk_pose(self):
        q = self.q
        pose = self.kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        return pose

    def get_chain(self):
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain("base_link", "ee_link")

    def get_jacobian(self):
        J = self.kdl_kin.jacobian(self.q)
        return J

    # def get_fk_pose(self):
    def get_ik_pose(self,pose):
        q_ik = self.kdl_kin.inverse(pose)  # inverse kinematics
        print "q_ik", q_ik
        return q_ik
    def test_fk_ik_jacob(self):
        q = self.q
        pose = self.kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        print pose

        q_ik = self.kdl_kin.inverse(pose, q)  # inverse kinematics
        print "q_ik", q_ik

        if q_ik is not None:
            pose_sol = self.kdl_kin.forward(q_ik)  # should equal pose
            print pose_sol

        # J = self.kdl_kin.jacobian(q)
        # print 'J:', J

def main():
    z_height=0.5
    radius=0.2
    ur5 = UR_robot()
    # pose=ur5.get_fk_pose()
    # print("pose is:",pose)

    q_ik1=[2.804412902508579, 5.335824801490162, 0.9954526241318554, 3.0935005351473617, 3.4787724046710093, 0.0]
    q_ik2=[2.804412902508579, 0.004550905073069474, 5.2877326830477305, 4.132494372648579, 3.4787724046710093, 0.0]
    q_ik3=[0.000703167047046982, 3.1370417485167232, 0.9954526241318566, 5.2922835881207995, 6.282482140131318, 0.0]
    q_ik4=[0.000703167047046982, 4.088953159279217, 5.28773268304773, 0.04809211844243201, 6.282482140131318]

    aubo_q=[1.50040841e-03, -2.83640237e+00, 1.53798406e+00, 1.29841831e+00, 1.50040840e-03, 3.14159265e+00]
    ur5.set_q(aubo_q)
    pose=ur5.get_fk_pose()
    pose[0,3]=-0.54
    pose[2,3]=0.51
    print("pose is:",pose)
    q_ik=ur5.get_ik_pose(pose)
    print(q_ik*180/pi)


if __name__ == "__main__":
    main()