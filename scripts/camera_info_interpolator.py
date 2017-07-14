#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from copy import deepcopy


class CamInfoInterpolator():

  def camInfoCallback(self, data):
    self.camInfo = data
    
  def depthCallback(self, data):
    if self.camInfo and data.header.stamp > self.camInfo.header.stamp: #If the image is more recent than the last received CameraInfo message...
      if data.height==self.camInfo.height and data.width==self.camInfo.width:
        self.camInfo.header.stamp = data.header.stamp
        self.pub.publish(self.camInfo)

  def __init__(self):
    rospy.init_node('camera_info_interpolator')
    rospy.loginfo("camera_info_interpolator node started")

    self.camInfo = None
    
    self.depthSub = rospy.Subscriber('camera/depth/image_raw', Image, self.depthCallback)
    self.camInfoSub = rospy.Subscriber('camera/depth/camera_info', CameraInfo, self.camInfoCallback)
    self.pub = rospy.Publisher('camera/depth/camera_info', CameraInfo, queue_size=10)

    rospy.spin()

  


if __name__ == '__main__':
  try:
    CamInfoInterpolator()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
