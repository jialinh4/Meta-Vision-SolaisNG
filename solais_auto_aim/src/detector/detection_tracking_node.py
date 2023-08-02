#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from your_package.srv import DetectionTracking
from communicator import UARTCommunicator
import config

def callback(data):
    yaw = data.data[0]
    pitch = data.data[1]
    
    rospy.wait_for_service('detection_tracking_service')
    try:
        detection_tracking = rospy.ServiceProxy('detection_tracking_service', DetectionTracking)
        response = detection_tracking(yaw, pitch)
        
        # 创建UARTCommunicator实例
        uart = UARTCommunicator(config)
        
        # 将新的yaw和pitch发送给下位机
        uart.process_one_packet(config.SEARCH_TARGET, response.new_yaw, response.new_pitch)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def listener():
    rospy.init_node('yaw_pitch_listener', anonymous=True)
    rospy.Subscriber('yaw_pitch_topic', Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
