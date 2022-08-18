#!/usr/bin/env python

import rospy
import random
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

sdf_cube = """<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="MODELNAME">
    <static>0</static>
    <link name="link">
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="stairs_collision0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <surface>
          <bounce />
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>10000000.0</kp>
              <kd>1.0</kd>
              <min_depth>0.0</min_depth>
              <max_vel>0.0</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="stairs_visual0">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>SIZEXYZ</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Wood</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.000000</linear>
        <angular>0.000000</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""

sdf_apriltag = """<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="MODELNAME">
    <static>0</static>
    <link name='tag_link'>
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.0246688</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0246688</iyy>
          <iyz>0</iyz>
          <izz>0.0441058</izz>
        </inertia>
      </inertial>
      <collision name='tag_collision'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.8 0.8 0.01</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name='main_Visual'>
        <geometry>
          <box>
          <size>0.8 0.8 0.01</size>
          </box>
        </geometry>
           <material>
          <script>
            <uri>model://Apriltag36_11_00000/materials/scripts</uri>
            <uri>model://Apriltag36_11_00000/materials/textures</uri>
            <name>Apriltag36_11_00000</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.000000</linear>
        <angular>0.000000</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""


def create_cube_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """Create a SpawnModelRequest with the parameters of the cube given.
    modelname: name of the model for gazebo
    px py pz: position of the cube (and it's collision cube)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the cube"""
    cube = deepcopy(sdf_cube)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    cube = cube.replace('SIZEXYZ', size_str)
    # Replace modelname
    cube = cube.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = cube
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req

def create_apriltag_request(modelname, px, py, pz, rr, rp, ry):
    req = SpawnModelRequest()

    apriltag = deepcopy(sdf_apriltag)
    # Replace modelname
    apriltag = apriltag.replace('MODELNAME', str(modelname))

    req.model_name = modelname
    req.model_xml = apriltag
    #req.model_xml = open('src/gazebo_ros_link_attacher/models/Apriltag36_11_00000/model.sdf', 'r').read()
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


if __name__ == '__main__':
    rospy.init_node('spawn_models')
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")
    

    x = random.uniform(10.0,12.0)
    y = random.uniform(7.0,9.0)
    # Spawn object 1
    rospy.loginfo("Spawning cube1")
    req1 = create_cube_request("cube1",
                              x, y, 0.40,  # position
                              0.0, 0.0, 0.0,  # rotation
                              0.8, 0.8, 0.4)  # size
    spawn_srv.call(req1)
    rospy.sleep(1.0)
  
    # Spawn apriltag
    rospy.loginfo("Spawning apriltag")
    req2 = create_apriltag_request("apriltag1",
                              x, y, 1.0,  # position
                              0.0, 0.0, 0.0,)  # rotation          
    spawn_srv.call(req2)
    rospy.sleep(1.0)
    
    '''
    rospy.loginfo("Spawning apriltag")
    req2 = create_apriltag_request("apriltag2",
                              16.5, 19.0, 0.2,  # position
                              0.0, 0.0, 0.0,)  # rotation          
    spawn_srv.call(req2)
    rospy.sleep(1.0)

    rospy.loginfo("Spawning apriltag")
    req2 = create_apriltag_request("apriltag3",
                              13.7, 19.0, 0.2,  # position
                              0.0, 0.0, 0.0,)  # rotation          
    spawn_srv.call(req2)
    rospy.sleep(1.0)'''
    
    # Link them
    
    rospy.loginfo("Attaching cube1 and apriltag")
    req = AttachRequest()
    req.model_name_1 = "cube1"
    req.link_name_1 = "link"
    req.model_name_2 = "apriltag1"
    req.link_name_2 = "tag_link"
    attach_srv.call(req)
    rospy.sleep(1.0)