#!/usr/bin/env python3
# Import essential libraries 
import requests 
import cv2 
import numpy as np 
import imutils
from ultralytics import YOLO
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

model = YOLO('yolov8n.pt')  # load a pretrained YOLOv8n detection model
#model = YOLO('best.pt')  # load a pretrained YOLOv8n detection model

# Replace the below URL with your own. Make sure to add "/shot.jpg" at last. 
url = "http://192.168.181.32:8080/shot.jpg"               

# Initialize the ROS node
rospy.init_node('object_detector')

# Create a ROS publisher
pub = rospy.Publisher('object_positions', Point, queue_size=10)

# Create a ROS Point message
point = Point()

# While loop to continuously fetching data from the URL 
while True:
    img_resp = requests.get(url)

    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    img = cv2.imdecode(img_arr, -1)
    img = imutils.resize(img, width=1000, height=1800)
        
    results = model(img)
    
    result = results[0]
    
    point.x = 0
    point.y = 0
    point.z = 0
    
    # Iterate over each detected object
    for box in result.boxes:
        
        ID = int (box.cls)
        if ID == 39:
            
            # Get bounding box coordinates
            coordinate = box.xyxy
            print(coordinate)
            # Calculate center coordinates
            center_x = int ((coordinate[0, 1] + coordinate[0, 2]) / 2)
            center_y = int ((coordinate[0, 1] + coordinate[0, 3]) / 2)
            # Draw a circle at the center of the object
            img = cv2.circle(img, (center_x, center_y), 5, (0, 255, 0), -1)
            # Draw a rectangle the object
            img = cv2.rectangle(img, (int (coordinate[0,1]), int (coordinate[0,1])), (int (coordinate[0,2]), int (coordinate[0,3])), (255,0,0), 2)
            
            point.x = center_x
            point.y = center_y
            point.z = 0

    # Publish the ROS Point message
    pub.publish(point)
       
    cv2.imshow("Android_cam", img)

    # Press Esc key to exit
    if cv2.waitKey(1) == 27:
        break


cv2.destroyAllWindows()

