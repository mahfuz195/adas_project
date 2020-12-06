import sys, traceback
import os
import argparse
import numpy as np
import cv2
import imutils
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry



from object_detection import detect_pedestrian


bag = rosbag.Bag("rosbag.bag")
bridge = CvBridge()


class Vehicle:
    def __init__(self):
        self.pose = []
        self.x = 0
        self.y = 0
        self.z = 0
        self.cur_time = 0
        self.prev_time = 0
        
    def setPose(self,pose):
        self.pose = pose
        #self.x = pose.position.x
        #self.y = pose.position.y
        #self.z = pose.position.z
        
    def getLocToString(self):
        ret = "[{:0.3f} , {:0.3f}, {:0.3f}]".format(self.x,self.y,self.z)
        return ret
    def getLoc(self):
        return self.x, self.y
          
          
class Pedestrian:
    def __init__(self):
        self.id = 0
        self.dist =-1
        self.bbox = []
        self.center = [0,0]
        self.azimuth = 0.0
        self.pos_x = 0
        self.pos_y = 0
        self.count = 4
        
    def setDistance(self,d):
        self.dist = d
        
    def getDistance(self):
        return self.dist
    
    def setBbox(self,x,y,w,h):
        self.bbox = [x,y,x + w, y + h]
        self.azimuth = (x+w/2)*(43.0/320.0) - 43.0
        self.center = [x+w/2,y+h/2]
    
    def getBbox(self):
        return self.bbox
    
    def getAzimuth(self):
        return self.azimuth
    
    def getCenter(self):
        return self.center
    
    def getPosition(self):
        return self.x,self,y
       
def calculateTTC(veh_pos,ped_dist):
    ttc = -1 #no collision    
    return ttc       


import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
xs = []
ys = []

def live(i,xs,ys):    
    xs= xs[-10:0]
    ys= ys[-10:0]
    ax.clear()
    ax.plot(xs,ys)
    plt.xtics(ha='right')
    plt.title('Distance Measurement')
    plt.ylabel('meter')
    
    

animation.FuncAnimation(fig,live,fargs=(xs,ys),interval=1000)    
    
""" list of topics"""
topic_raw_image = "/D435I/color/image_raw"
topic_dep_image = "/D435I/depth/image_rect_raw"
topic_odom = "/T265/odom/sample"
topic_camera_info = "/D435I/color/camera_info"
""" collect data from these topics """

pedestrian = Pedestrian()
vehicle = Vehicle()
reference_time = None
img = None
d_color_map =None
count = 0
q_depth_image = []
q_odom_info = []

def get_depth_image(ref_time):
    #print ('total queue depth size:', len(q_depth_image))
    
    if(len(q_depth_image)==0): 
        return []
    else :
        ret = q_depth_image[0][1]
        while(len(q_depth_image)!=0):
            ele = q_depth_image[0]
            ret = ele[1]
            #print ('given ref time ' , ref_time, ' q front time ', ele[0])
            if(ele[0]<=ref_time):
                q_depth_image.pop(0)    
            else :
                return ret
        
        return ret    

""" tracking """
#tracker = cv2.TrackerKCF_create()
#tracker = cv2.MultiTracker_create()
tracker = cv2.TrackerCSRT_create()
prev_dist = 0.0
cur_dist = 0.0
      
#fourcc = cv2.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 20.0, (1280,480))

for topic,msg, t in bag.read_messages(topics=[topic_raw_image,topic_odom,topic_dep_image]):
    cur_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
    try:
        """
        color image dataset. 
        """
        if(topic == topic_raw_image):
            reference_time = cur_time
            #print ("raw image")
            cv_img_color = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv_img_color = cv2.cvtColor(cv_img_color,cv2.COLOR_BGR2RGB)
            img, bbox = detect_pedestrian(cv_img_color)
            
            #track_d_image = np.dstack((d_image,d_image,d_image))
            
            
            if(len(bbox)>0):
                #print ("pedestrian detected: "+ str(pedestrian.count+1))
                x,y,w,h = bbox[0]
                pedestrian.setBbox(x,y,w,h)
                ped_info = "[x: {:0.3f}, y: {:0.3f}, {:0.3f} deg]".format(pedestrian.pos_x,pedestrian.pos_y,pedestrian.getAzimuth())
                cv2.putText(img,ped_info,(20,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
                
                #cv2.rectangle(d_color_map, (x-10,y-10), (x+10,y+10), (0,255,0), -1)
                """
                c_x = int(x + w/2)
                c_y = int(y + h/2)
                
                #track_box = (x,y,x+w,y+h)
                track_box = (c_x-10,c_y-10,c_x+10,c_y+10)
                #cv2.rectangle(d_color_map, (x,y), (x+w,y+h), (0,255,0), -1)
                
                if(pedestrian.count>=0):
                    #d_track_image = np.dstack((d_image, d_image,d_image)) 
                    tracker.init(cv_img_color,(track_box))
                    print ("init tracking tracking")
                """
                pedestrian.count = min(5,pedestrian.count+1)
            
            
            else:
                if(pedestrian.count<=0): 
                    pedestrian.count = 0
                    pedestrian.setBbox(0,0,0,0)
                    #tracker.init(cv_img_color,(0,0,0,0))
                    #tracker = cv2.TrackerCSRT_create()
                    #print ("resetting tracking")
                    
                else:
                    pedestrian.count-=1
            
            #success, box = tracker.update(cv_img_color)
            d_image = get_depth_image(reference_time-1.55)
            dist = 0.0
            if(len(d_image)!=0):
                x1,y1,x2,y2 = pedestrian.getBbox()
                d_color_map = cv2.applyColorMap(cv2.convertScaleAbs(d_image,alpha=0.03),cv2.COLORMAP_JET)
                cv2.rectangle(d_color_map, (x1,y1), (x2,y2), (0,255,0), 2)
                c_x = (x1+x2)//2
                c_y = (y1+y2)//2
                dist = np.array(d_image[c_x-10:c_x+10,c_y-10:c_y+10]).mean()
                cv2.rectangle(d_color_map, (c_x-10,c_y-10), (c_x+10,c_y+10), (0,255,0), -1)

                
                
            cv2.putText(img,"Dist={:0.3f}".format(dist),(20,350), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
            v_loc = "veh pos:" + vehicle.getLocToString()
            cv2.putText(img, v_loc, (20,450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
            
            images = np.hstack((img, d_color_map)) 
            
            cv2.imshow('Collision Warning!',images)
            out.write(images)    
            
            """    
            if(len(bbox)==0):
                if(pedestrian.count>=0): 
                    pedestrian.count-=1
                    pedestrian.setBbox(0,0,0,0)
            else:
                pedestrian.count+=1
                pedestrian.count = min(4,pedestrian.count)
                
            if(len(bbox)>0):
                (ba,bb,bc,bd) = bbox[0]
            
                "
                tracker.init(cv_img_color,(ba,bb,bc,bd))
                
            success, box = tracker.update(cv_img_color)
            
            if(success):
                (x,y,w,h) = [int(v) for v in box] 
                cv2.rectangle(cv_img_color, (x,y), (x+w,y+h), (0,255,0), 2)
                cv2.imshow('Tracked',cv_img_color)
            
            
            v_loc = "veh pos:" + vehicle.getLocToString()
            cv2.putText(img, v_loc, (20,450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
            if(len(bbox)>0):
                #print ('pedestian detected')
                x,y,w,h = bbox[0]
                pedestrian.setBbox(x,y,w,h)
                ped_info = "[x: {:0.3f}, y: {:0.3f}, {:0.3f} deg]".format(pedestrian.pos_x,pedestrian.pos_y,pedestrian.getAzimuth())
                cv2.putText(img,ped_info,(20,400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
                
            
            d_image = get_depth_image(reference_time-1.5)
            if(len(d_image)!=0):
                d_color_map = cv2.applyColorMap(cv2.convertScaleAbs(d_image,alpha=0.03),cv2.COLORMAP_JET)
               
                c_x = int((pedestrian.bbox[0]+pedestrian.bbox[2])/2.0)
                c_y = int((pedestrian.bbox[1]+pedestrian.bbox[3])/2.0)
                
                #print ('the distance is :', d_image[c_x,c_y])
                cv2.rectangle(d_color_map, (pedestrian.bbox[0],pedestrian.bbox[1]), (pedestrian.bbox[2],pedestrian.bbox[3]), (0,255,0), 2)
                
                cv2.rectangle(d_color_map, (c_x-10,c_y-10), (c_x+10,c_y+10), (0,255,0), -1)
                
                dist = np.array(d_image[c_x-10:c_x+10,c_y-10:c_y+10]).mean()
                
                cv2.putText(img,"Dist={:0.3f}".format(dist),(20,350), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
                
                images = np.hstack((img, d_color_map)) 
                
                cv2.imshow('Image',images)
            else:
                print ('depth image not found!')
        """    
                
        elif (topic == topic_dep_image):    
            cv_img = bridge.imgmsg_to_cv2(msg, "32FC1")
            d_color_map = cv2.applyColorMap(cv2.convertScaleAbs(cv_img,alpha=0.03),cv2.COLORMAP_JET)
            
            x,y = pedestrian.getCenter()
            #images = np.hstack((img, d_color_map))
            dist = cv_img[y,x]
            q_depth_image.append([cur_time,cv_img])
            
            """
            angle = pedestrian.getAzimuth()
            x,y = pedestrian.getCenter()
                
            v_x, v_y = vehicle.getLoc()
            x_pos = v_x + dist*np.cos(angle) 
            y_pos = v_y + dist*np.sin(angle) 
            
            pedestrian.pos_x = x_pos
            pedestrian.pos_x = y_pos
            
                
            xs.append(count)
            ys.append(cv_img[y,x])
            """
            #count+=1
            
        elif (topic== topic_odom):
            #vehicle.setPose(msg.pose.pose)
            #print ()
            v_x = msg.twist.twist.linear.x
            v_y = msg.twist.twist.linear.y
            
            vehicle.cur_time = cur_time
            
            if(vehicle.prev_time==0):
                vehicle.prev_time = vehicle.cur_time 
            else :
                dt = vehicle.cur_time - vehicle.prev_time
                vehicle.x = vehicle.x + v_x*dt
                vehicle.y = vehicle.y+ v_y*dt
                vehicle.prev_time = vehicle.cur_time 
                
        count+=1
        ch = cv2.waitKey(1)
        if ch == 27:
            break

    except Exception as e:
        print ('exception', e)
        traceback.print_exc()
        continue
out.release()
bag.close()
cv2.destroyAllWindows()

