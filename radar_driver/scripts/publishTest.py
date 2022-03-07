#!/usr/bin/env python

import rospy
from copy import *
from ars430_ros_publisher.msg import  EthCfgRx
from ars430_ros_publisher.msg import  RadarPacket 
from ars430_ros_publisher.msg import  SensorCfgRx
from ars430_ros_publisher.msg import  SensorStatus
from ars430_ros_publisher.msg import  RadarDetection

# initialise
 
EthCfgRx_pub        = rospy.Publisher('EthCfgRx_data', EthCfgRx, queue_size=50)
radar_imageTX_pub   = rospy.Publisher('RadarPacket_data',radar_imageTX, queue_size=50)
SensorCfgRx_pub     = rospy.Publisher('SensorCfgRx_data', SensorCfgRx, queue_size=50)
sensor_statusTx_pub = rospy.Publisher('SensorStatus_data',sensor_statusTx, queue_size=50)


rospy.init_node('publish_ars430',anonymous=True)
r = rospy.Rate(5)

#setup messages to publish
EthCfgRx_msg        = EthCfgRx()
radar_imageTX_msg   = radar_imageTX()
SensorCfgRx_msg     = SensorCfgRx()
sensor_statusTx_msg = sensor_statusTx()
RadarDetection_msg  = RadarDetection()



radar_imageTX_msg.RDI_Far0_List.append(copy(RadarDetection_msg))
radar_imageTX_msg.RDI_Far0_List[0].f_Range = 4
radar_imageTX_msg.RDI_Far0_List[0].f_VrelRad = 9
radar_imageTX_msg.RDI_Far0_List.append(copy(RadarDetection_msg))
print(RadarDetection_msg)
radar_imageTX_msg.RDI_Far0_List[1].f_Range = 7
radar_imageTX_msg.RDI_Far0_List[1].f_VrelRad = 1
radar_imageTX_msg.RDI_Far0_List.append(copy(RadarDetection_msg))


sensor_statusTx_msg.SensorStatus_CRC = 4
sensor_statusTx_msg.SensorStatus_Len = 3
sensor_statusTx_msg.SensorStatus_SerialNumber = [4,3,4,6,7]


#while not rospy.is_shutdown():
rospy.loginfo("Publish SensorCfgRx Started!")
#rospy.loginfo(SensorCfgRx_msg)
rospy.loginfo(sensor_statusTx_msg)

#SensorCfgRx_pub.publish(SensorCfgRx_msg)
radar_imageTX_pub.publish(radar_imageTX_msg)
r.sleep()

rospy.loginfo("Publish SensorCfgRx Started!")
rospy.loginfo(sensor_statusTx_msg)
#rospy.loginfo(SensorCfgRx_msg)
#rospy.loginfo(radar_imageTX_msg.RDI_Far0_List)

#SensorCfgRx_pub.publish(SensorCfgRx_msg)
#radar_imageTX_pub.publish(radar_imageTX_msg)
sensor_statusTx_pub.publish(sensor_statusTx_msg)
r.sleep()

if __name__ == '__main__':
	try:
		rospy.loginfo("Publish SensorCfgRx Ended!")
	except rospy.ROSInterruptException: pass
