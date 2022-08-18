#!/usr/bin/env python

from curses import intrflush
from time import sleep
from xmlrpc.client import Boolean, boolean
import rospy
from quadcopter_landing.srv import Attach, AttachRequest, AttachResponse
from std_msgs.msg import Int16


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    state_pub = rospy.Publisher("/delivery_state", Int16, queue_size=10)
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "iris_leader"
    req.link_name_1 = "iris_leader/base_link"
    req.model_name_2 = "cube1"
    req.link_name_2 = "link"

    attach_srv.call(req)
    # From the shell:
    """
rosservice call /link_attacher_node/attach "model_name_1: 'cube1'
link_name_1: 'link'
model_name_2: 'cube2'
link_name_2: 'link'"
    """

    state = 1
    sleep(10)
    rospy.loginfo("Ready to delivery!!")

    state_pub.publish(state)
