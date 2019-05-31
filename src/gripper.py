#!/usr/bin/env python

from rg6.srv import Grip, GripResponse, GripperForce, GripperForceResponse
from rg6.msg import Gripper
from ur_msgs.srv  import SetIO
from ur_msgs.msg  import IOStates, ToolDataMsg
import rospy
import threading
import math

class Atomic():
    def __init__(self, value):
        self._value = value
        self._lock = threading.Lock()

    @property
    def value(self):
        with self._lock:
            return self._value

    @value.setter
    def value(self, v):
        with self._lock:
            self._value = v
            return self._value


gripper_width = Atomic(0.0)

pub = rospy.Publisher('gripper', Gripper, queue_size=1)

def handle_grip(req): 
    handled = False
    grip = 1 if req.grip else 0
    try:
        setIO = rospy.ServiceProxy('/set_io', SetIO)
        resp = setIO(1, 8, grip) # (1 - indicates its a flag, 8 is the digital io pin for the gripper, state of the flag)
        handled = resp.success
    except rospy.ServiceException as e:
        rospy.logwarn ("Service call failed: %s"%e)

    return GripResponse(handled)

def handle_gripper_force(req): 
    handled = False
    grip = 1 if req.highForce else 0
    try:
        setIO = rospy.ServiceProxy('/set_io', SetIO)
        resp = setIO(1, 9, grip) # (1 - indicates its a flag, 8 is the digital io pin for the gripperForce, state of the flag)
        handled = resp.success
    except rospy.ServiceException as e:
        rospy.logwarn ("Service call failed: %s"%e)

    return GripperForceResponse(handled)

def tool_data_callback(data):
    gripper_width.value = (math.floor(((data.analog_input2-0.026)*110/2.976)*10))/10

def io_callback(data):
    gripper = Gripper()

    gripper.grippedDetected = data.digital_in_states[8].state  
    gripper.moving          = not data.digital_in_states[9].state    
    gripper.width           = gripper_width.value

    pub.publish(gripper)

def gripper_server():
    rospy.init_node('rg6_gripper')
    rospy.loginfo( "Waiting for UR cubicle")
    rospy.wait_for_service('/set_io')

    s = rospy.Service('grip', Grip, handle_grip)
    s = rospy.Service('gripper_force', GripperForce, handle_gripper_force)
    rospy.Subscriber("/io_states", IOStates, io_callback)
    rospy.Subscriber("/tool_data", ToolDataMsg, tool_data_callback)
    rospy.loginfo( "Ready to grip")
    rospy.spin()


if __name__ == "__main__":
    gripper_server()
