#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from curses import intrflush
from time import sleep
from std_msgs.msg import Int16


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    #state_pub = rospy.Publisher("/delivery_state", Int16, queue_size=10)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    req = AttachRequest()
    req.model_name_1 = "cube1"
    req.link_name_1 = "link"
    req.model_name_2 = "iris_leader1"
    req.link_name_2 = "iris_leader1/base_link"
    attach_srv.call(req)

    req.model_name_1 = "cube1"
    req.link_name_1 = "link"
    req.model_name_2 = "iris_leader2"
    req.link_name_2 = "iris_leader2/base_link"
    attach_srv.call(req)

    req.model_name_1 = "cube1"
    req.link_name_1 = "link"
    req.model_name_2 = "iris_follower1"
    req.link_name_2 = "iris_follower1/base_link"
    attach_srv.call(req)

    req.model_name_1 = "cube1"
    req.link_name_1 = "link"
    req.model_name_2 = "iris_follower2"
    req.link_name_2 = "iris_follower2/base_link"
    attach_srv.call(req)
     
    '''
    state = 1
    sleep(10)
    rospy.loginfo("Ready to delivery!!")
    state_pub.publish(state)'''


