import cv2
import sys
import rospy
import math
import time
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt

Video_Location = "/dev/video0"

def get_center(bbox):
	cx = bbox[0] + bbox[2]/2
	cy = bbox[1] + bbox[3]/2
	return cx, cy

def rectpts(bbox):
        return (math.floor(bbox[0]), math.floor(bbox[1])) , (math.floor(bbox[0] + bbox[2]), math.floor(bbox[1] + bbox[3]))


def get_relpos(frame,bbox):
	cx, cy = get_center(bbox)
	rows = float(frame.shape[0])
	cols = float(frame.shape[1])
	x = (cx/(0.5*cols) -1)
	y = (cy/(0.5*rows) -1)
	return x, y

def publishinfo(pub_flt, pub_twst, bbox, vel, omega):
	fltmsg = Float64MultiArray()
	fltmsg.data = list(bbox)
	pub_flt.publish(fltmsg)
	twstmsg = Twist()
	twstmsg.linear.x = vel
	twstmsg.angular.z = omega
	pub_twst.publish(twstmsg)

def get_diag(frame, bbox):
	rows = float(frame.shape[0])
	cols = float(frame.shape[1])
	x = bbox[2]/0.5/cols
	y = bbox[3]/0.5/rows
	return (x**2 + y**2)**0.5

def gimme_cmdvel(frame, bbox, diag):
	x,y = get_relpos(frame,bbox)
	omega = 2 * x
	new_diag = get_diag(frame, bbox)
	vel = 2 * (diag - new_diag)
	return vel, omega

if __name__ == '__main__' :
    print(cv2.__version__)
    pub_flt = rospy.Publisher('rect_points', Float64MultiArray, queue_size=10)
    pub_twst = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('camandcmd_node', anonymous=True)
    tracker_type = "CSRT"	
    tracker = cv2.TrackerCSRT_create()
    video = cv2.VideoCapture(Video_Location)
 
    if not video.isOpened():
        print("Could not open video")
        sys.exit()

    ok, frame = video.read()
    print(frame.shape)
    #cv2.imshow('',frame)
    bbox = cv2.selectROI(frame, False)
    print(bbox)
    diag = get_diag(frame, bbox) 
    ok = tracker.init(frame, bbox)

    ok, frame = video.read()
    cxes = []
    cys = []
    vels = []
    
    prev_time = time.time()
    while not rospy.is_shutdown():
        ok, bbox = tracker.update(frame)
        if ok:
            rectpt1, rectpt2 = rectpts(bbox)
            cv2.rectangle(frame, rectpt1, rectpt2, (255,255,0), 2, 1)
        else :
            cv2.putText(frame, "Tracking failure", (100,80), cv2.FONT_HERSHEY_PLAIN, 1,(0,0,255),2)
        cv2.putText(frame, tracker_type, (100,20), cv2.FONT_HERSHEY_PLAIN, 1, (50,170,50),2);
        cur_time = time.time()
        fps = 1/(cur_time - prev_time)
        prev_time = cur_time
        cv2.putText(frame,"FPS: "+str(int(fps)), (100,40), cv2.FONT_HERSHEY_PLAIN, 1, (150,100,250), 2)
        cv2.imshow("Tracking", frame)
        vel, omega = gimme_cmdvel(frame, bbox, diag)
        vels.append(vel)
        publishinfo(pub_flt, pub_twst, bbox, vel, omega)
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
        ok, frame = video.read()
        cx, cy = get_relpos(frame, bbox)
        cxes.append(cx)
        cys.append(cy)
    #plt.plot(cxes, cys)
    #plt.axis([-1, 1, -1, 1])
    #plt.show()
    plt.plot(cxes)
    plt.show()
    #plt.plot(vels)
    #plt.show()    
        

	
