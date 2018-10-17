#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import numpy as np

class Follower():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/follower/mono_b/camera1/image_raw', Image, self.image_callback)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        dist= np.matrix([0.0, 0.0, 0.0, 0.0, 0.0])
        K= [[381.36246688113556, 0.0, 320.5], [0.0, 381.36246688113556, 240.5], [0.0, 0.0, 1.0]]
        R= [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        #P: [[381.36246688113556, 0.0, 320.5], [-26.69537268167949, 0.0, 381.36246688113556], [240.5, 0.0, 0.0], [0.0, 1.0, 0.0]]
        mtx= np.matrix([[381.36246688113556, 0.0, 320.5], [-26.69537268167949, 0.0, 381.36246688113556], [240.5, 0.0, 0.0]])
        (rows,cols,channels) = frame.shape
        print(rows,cols,channels)
        #if cols > 60 and rows > 60 :
        #    cv2.circle(cv_image, (50,50), 10, 255)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if np.all(ids!=None):
             rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
             font = cv2.FONT_HERSHEY_SIMPLEX
             for i in range(0, ids.size):
                 aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)  # Draw Axis
             aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers

             ###### DRAW ID #####
             strg = ''
             for i in range(0, ids.size):
                 strg += str(ids[i][0])+', '

             cv2.putText(frame, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


        cv2.imshow("Image window", frame)
        cv2.waitKey(3)
        
    

rospy.init_node('Follower')
follower = Follower()
print("create a follower node")
rospy.spin()


