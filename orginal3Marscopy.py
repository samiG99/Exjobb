import rosbag 
import cv2
import numpy as np 

arr = []
global frame
ff = True
def readBagFile():
    readingBagFile = rosbag.Bag("2022-08-26-15-20-17.bag")
    for topic, msg, t in readingBagFile.read_messages(topics=['/robot/fisheye_front/image_color']):
        arr.append(msg)
        #print("Message",msg)
    
    if len(arr)<10:
        raise ValueError ("the size of array is not enough")

    readingBagFile.close()

def ConvertTOFrame(msg):
    #convert the msg data to binary data, the string to binary, binary to bytearray  
    BIN_DATA = bytearray(msg.data)
    
    #converting the msg to frame 
    frame = np.array(BIN_DATA, dtype=np.uint8)
    frame = frame.reshape(msg.height, msg.width, -1)
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
        frame_without_box[690:780, 934:1080] = 0
        # Calculate the absolute difference between the current frame and the previous frame
        absolut_difference = cv2.absdiff(frame_without_box, cv2.convertScaleAbs(previus_frame))
        absolut_difference1 = cv2.absdiff(frame, cv2.convertScaleAbs(previus_frame))
        
        # Find contours of the moving object
        gray = cv2.cvtColor(absolut_difference1, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY) # if we have something less than 25 
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        box_size = [(1000,720), (1000, 720), (1000, 720), (1000, 720)]
        center = (int((box_size[0][0] + box_size[1][0]) / 2), int((box_size[0][1] + box_size[2][1]) / 2))
        pt1 = (center[0] - 65, center[1] - 30)
        pt2 = (center[0] + 80, center[1] + 60)
       
        # Draw the box rectangle
        cv2.rectangle(absolut_difference, pt1, pt2, (0, 255, 0), 2)

        # looking if there is some object moving in the frame, detect the obejct
        if len(contours)>0:
            #finding the contur with largest area
            largest_contour = max(contours, key=cv2.contourArea)
            # Calculate the moments of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(absolut_difference,  (cx, cy),30, (0, 0, 255), -1)
        cv2.imshow("change in foreground", cv2.resize(absolut_difference, (1000,1000)))
        cv2.imshow("processTheFrame", cv2.resize(frame, (600,500)))
        key = cv2.waitKey(80)
        if key == 27:
            exit()
    cv2.destroyAllWindows()




readBagFile()
objectdetection()

    
    