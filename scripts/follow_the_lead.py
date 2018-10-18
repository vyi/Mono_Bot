#!/usr/bin/env python
import rospy 
import math
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import numpy as np


def inversePerspective(rvec, tvec):
    """ Applies perspective transform for given rvec and tvec. """
    rvec, tvec = rvec.reshape((3, 1)), tvec.reshape((3, 1))
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = cv2.Rodrigues(R)

    invTvec = invTvec.reshape((3, 1))
    invTvec = invTvec.reshape((3, 1))
    return invRvec, invTvec

def euclideanDistanceOfTvecs(tvec1, tvec2):
    return math.sqrt(math.pow(tvec1[0]-tvec2[0], 2) + math.pow(tvec1[1]-tvec2[1], 2) + math.pow(tvec1[2]-tvec2[2], 2))

def euclideanDistanceOfTvec(tvec):
    return euclideanDistanceOfTvecs(tvec, [0, 0, 0])
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

class Follower():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/follower/mono_b/camera1/image_raw', Image, self.image_callback)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
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
        #mtx= np.matrix([[381.36246688113556, 0.0, 320.5], [-26.69537268167949, 0.0, 381.36246688113556], [240.5, 0.0, 0.0]])
        mtx = np.copy(K)
        (rows,cols,channels) = frame.shape
        print("rows:{}, columns:{}, channels:{}".format(rows,cols,channels))
        #if cols > 60 and rows > 60 :
        #    cv2.circle(cv_image, (50,50), 10, 255)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
		#### CORNERS FETCHING
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        print("conrners {}, rejected {}".format(corners, rejectedImgPoints))
        
        if np.all(ids!=None):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.18, mtx, dist)
            font = cv2.FONT_HERSHEY_SIMPLEX
            for i in range(0, ids.size):
                aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)  # Draw Axis
            aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers
            rv, tv = inversePerspective(rvec, tvec)

            rotM = np.zeros(shape=(3,3))
                        
            print("norm: {}".format(cv2.norm(tvec[0][0])))
            # RotM, _ = cv2.Rodrigues(rvec[0])
            # print("RotM : {}, vec: {}".format(RotM, rvec[0]))
            # ypr = rotationMatrixToEulerAngles(RotM)
            # print("ypr: {}".format(ypr))
            print("rv:{}, {}".format(rv, math.atan2(rv[2][0],rv[1][0])))
            #print("rv:{}, {}".format(rv, rv.shape))
            ###### DRAW ID #####
            strg = ''
            for i in range(0, ids.size):
                strg += str(ids[i][0])+', '

            cv2.putText(frame, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
            cv2.putText(frame, "Detected Pose: {} {}".format(rvec, tvec), (0,64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            print("Detected Pose: {} {}".format(rvec, tvec))
        cv2.imshow("Image window", frame)
        cv2.waitKey(3)
        
    

rospy.init_node('Follower')
follower = Follower()
print("create a follower node")
rospy.spin()


