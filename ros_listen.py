import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry

bag = rosbag.Bag("sensorfeeds-forklift-driving-into-Pedestrian.bag")
bridge = CvBridge()

img = 0

""" list of topics"""
topic_raw_image = "/D435I/color/image_raw"
topic_dep_image = "/D435I/depth/image_rect_raw"
topic_odom = "/T265/odom/sample"

""" collect data from these topics """

for topic,msg, t in bag.read_messages(topics=["/D435I/color/image_raw","/T265/odom/sample"]):

    print (t)
    print ('-------------------------')
    print (msg)
    print ('-------------------------')
    print (t)
    print ('-------------------------')

    if(topic == topic_raw_image):
        print ("raw image")
    elif (topic == topic_dep_image):
        print ("dep image")
    elif (topic== topic_odom):
        print ('odo data')

    #print (type(msg))
    #cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    #cv2.imwrite("imgs/frame%06i.png"%img, cv_img)
    #print ("Wrote image")
    #img+=1
    #break

bag.close()

