#! /usr/bin/env python

# Import the module
import sys
sys.path.append("/home/zy/catkin_ws/src/hrl-kdl")
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy
from frompitoangle import *

def jacobian_generation(q):
    robot = URDF.from_xml_file("/home/zy/catkin_ws/src/aubo_robot-master/aubo_description/urdf/aubo_i5.urdf")
    tree = kdl_tree_from_urdf_model(robot)
    # print tree.getNrOfSegments()
    # chain = tree.getChain("base_link", "wrist3_Link")
    # print chain.getNrOfJoints()

    # forwawrd kinematics
    kdl_kin = KDLKinematics(robot, "base_link", "wrist3_Link")
    pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 matrix)
    # print pose
    # print type(pose)
    J = kdl_kin.jacobian(q)
    # print 'J:', J
    # print len(J)
    return J

if __name__ == "__main__":
    try:
        q = [0, 0, 1,0,1,0]
        jacobian_mat=jacobian_generation(q)
    except rospy.ROSInterruptException:
        pass