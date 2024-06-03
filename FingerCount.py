#!/usr/bin/env python3

import cv2
import mediapipe as mp
import time

# pip install python-time
# pip install opencv-python
# pip install mediapipe

import requests
import numpy as np 
import imutils

import rospy
from geometry_msgs.msg import Twist
import playsound

pub = rospy.Publisher('our_cmd_vel', Twist, queue_size=10)
rospy.init_node('finger_counter', anonymous=True)
rate = rospy.Rate(10) # 10hz

    
mp_draw = mp.solutions.drawing_utils
mp_hand = mp.solutions.hands
tipIds = [4, 8, 12, 16, 20]

url = 'http://192.168.181.32:8080/shot.jpg'   # if you want to access cam through URL  # install ip webcame pro   yehia

#video = cv2.VideoCapture(url)




with mp_hand.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
    while True:
        
        img_resp = requests.get(url)

        img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
        video = cv2.imdecode(img_arr, -1)
        video = imutils.resize(video, width=1000, height=1800)

        image = video
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = hands.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        lmList = []
        if results.multi_hand_landmarks:
            for hand_landmark in results.multi_hand_landmarks:
                myHands = results.multi_hand_landmarks[0]
                for id, lm in enumerate(myHands.landmark):
                    h, w, c = image.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lmList.append([id, cx, cy])
                mp_draw.draw_landmarks(image, hand_landmark, mp_hand.HAND_CONNECTIONS)
        fingers = []
        if len(lmList) != 0:
            if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
                fingers.append(1)
            else:
                fingers.append(0)
            for id in range(1, 5):
                if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            total = fingers.count(1)
            

            if total == 0:
                cv2.putText(image, "No Finger", (45, 375), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)

            elif total == 1:
                cv2.putText(image, "1 Finger", (45, 375), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)

            elif total == 2:
                cv2.putText(image, "2 Finger", (45, 375), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)

            elif total == 3:
                cv2.putText(image, "3 Finger", (45, 375), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)

            elif total == 4:
                cv2.putText(image, "4 Finger", (45, 375), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)

            elif total == 5:
                cv2.putText(image, "5 Finger", (45, 375), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
	    
            TwistValue = Twist()
            TwistValue.linear.z = float(total)
            pub.publish(TwistValue)
            rate.sleep()
            
            cv2.imshow("Frame", image)
            
            if total == 1:
                playsound.playsound("/home/mariam/One_printer.mp3", True)

            elif total == 2:
                playsound.playsound("/home/mariam/Two_meeting.mp3", True)

            elif total == 3:
                playsound.playsound("/home/mariam/Three_office.mp3", True)
            
        cv2.imshow("Frame", image) 
        
        
                
        k = cv2.waitKey(1)
        if k == ord('q'):
            break
video.release()
cv2.destroyAllWindows()
