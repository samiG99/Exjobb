import rosbag 
import cv2
from cv_bridge import CvBridge
import numpy as np 

arr = []
depth_arr = []
global frame
ff = True
moving_object = None


def readBagFile():
    readingBagFile = rosbag.Bag("2022-08-26-15-20-17.bag")
    for topic, msg, t in readingBagFile.read_messages(topics=['/robot/k4a_top/depth/image_rect']):
        arr.append(msg)
    
    if len(arr)<10:
        raise ValueError ("the size of array is not enough")

    readingBagFile.close()


def ConvertTOFrame(msg):
    #convert the msg data to binary data, the string to binary, binary to bytearray  
    BIN_DATA = bytearray(msg.data)
    
    #converting the msg to frame 
    frame = np.array(BIN_DATA, dtype=np.uint8)
    frame = frame.reshape(msg.height, msg.width, -1)
    #print(frame.shape)
    return frame

def objectdetection():
    previus_frame = None
    for msg in arr:
        frame = ConvertTOFrame(msg)
        if previus_frame is None:
            previus_frame = np.float32(frame)
        
        # Update the previous frame
        previus_frame = cv2.accumulateWeighted(frame, previus_frame, 0.3)
        
        frame_without_box = frame.copy()
        frame_without_box[110:170, 110:165] = 0
        # Calculate the absolute difference between the current frame and the previous frame
        absolut_difference = cv2.absdiff(frame_without_box, cv2.convertScaleAbs(previus_frame))
        absolut_difference1 = cv2.absdiff(frame, cv2.convertScaleAbs(previus_frame))
        
        # Find contours of the moving object
        gray = cv2.cvtColor(absolut_difference1, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY) # if we have something less than 25 
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        #box_size = [(1000,720), (1000, 720), (1000, 720), (1000, 720)]
        #center = (int((box_size[0][0] + box_size[1][0]) / 2), int((box_size[0][1] + box_size[2][1]) / 2))
        pt1 = (110 , 110 )
        pt2 = (167 , 167 )
       
        # Draw the box rectangle
        cv2.rectangle(absolut_difference, pt1, pt2, (0, 255, 0), 1)

        # looking if there is some object moving in the frame, detect the obejct
        if len(contours)>0:
            #finding the contur with largest area
            largest_contour = max(contours, key=cv2.contourArea)
            largest_contour_size = cv2.contourArea(largest_contour)
            if largest_contour_size <= 100:
                largest_contour = 0
                #print(largest_contour_size)
            # Calculate the moments of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(absolut_difference,  (cx, cy), 5, (0, 0, 255), -1)
                # Create a mask for the moving object
                mask = np.zeros_like(gray)
                cv2.drawContours(mask, [largest_contour], -1, (255, 255, 255), -1)
                # Keep only the moving object in the difference image
                #moving_object = cv2.bitwise_and(absolut_difference, absolut_difference, mask=mask)
                

        else:
            moving_object = absolut_difference
        
        cv2.imshow("change in foreground", cv2.resize(moving_object, (800,600)))
        cv2.imshow("change in ", cv2.resize(absolut_difference, (800,600)))
        #cv2.rectangle(moving_object, pt1, pt2, (0, 255, 0), 2)
        #cv2.imshow("processTheFrame", cv2.resize(frame, (600,500)))
        key = cv2.waitKey(0)
        if key == 27:
            exit()
    cv2.destroyAllWindows()






readBagFile()
objectdetection()
