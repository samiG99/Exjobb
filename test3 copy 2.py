import rosbag 
import cv2
import numpy as np 

arr = []

def readBagFile():
    readingBagFile = rosbag.Bag("2022-08-26-15-18-39.bag")
    for topic, msg, t in readingBagFile.read_messages(topics=['/robot/fisheye_front/image_color']):
        arr.append(msg)
        #print("Message",msg)
    
    if len(arr)<10:
        raise ValueError ("the size of array is not enough")

    readingBagFile.close()

def ConvertTOFrame():
    previus_frame = None
    for msg in arr:
        #convert the msg data to binary data, the string to binary, binary to bytearray  
        BIN_DATA = bytearray(msg.data)
        
        #converting the msg to frame 
        frame = np.array(BIN_DATA, dtype=np.uint8)
        frame = frame.reshape(msg.height, msg.width, -1)
        
       
        if previus_frame is None:
            previus_frame = np.float32(frame)
            continue

        
        # Update the previous frame
        previus_frame = cv2.accumulateWeighted(frame, previus_frame, 0.5)
        
        frame_without_box = frame.copy()
        frame_without_box[690:780, 934:1080] = 0

        #for i in range(frame.shape[0]): # 0-1539
        #    for j in range(frame.shape[1]): # 0-1700
        #        if not ((690 < i < 780) and (934 < j < 1080)):
        #            #frame_without_box.append([i][j]) = frame
        #            frame_without_box.append(frame[i][j])

        # Calculate the absolute difference between the current frame and the previous frame
        absolut_difference = cv2.absdiff(frame_without_box, cv2.convertScaleAbs(previus_frame))

        # Find contours of the moving object
        #gray = cv2.cvtColor(absolut_difference, cv2.COLOR_BGR2GRAY)
        #_, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY) # if we have something less than 25 
        #contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        box_size = [(1000,720), (1000, 720), (1000, 720), (1000, 720)]
        center = (int((box_size[0][0] + box_size[1][0]) / 2), int((box_size[0][1] + box_size[2][1]) / 2))
        pt1 = (center[0] - 65, center[1] - 30)
        pt2 = (center[0] + 80, center[1] + 60)

        #print(center)

        # Draw the box rectangle
        cv2.rectangle(absolut_difference, pt1, pt2, (0, 255, 0), 2)

        #for cnt in contours:
        #    M = cv2.moments(cnt)
        #    if M["m00"] != 0:
        #        cx = int(M["m10"] / M["m00"])
        #        cy = int(M["m01"] / M["m00"])
        #        cv2.circle(absolut_difference,  (cx, cy), 5, (0, 0, 255), -1)

        # Display frames
        #abs = cv2.cvtColor(absolut_difference,cv2.COLOR_BGR2GRAY)
        #print(absolut_difference.shape)
        
        #x = 0
        #y = 0
        #for abs in absolut_difference:
        #    y = y + 1
        #    x = 0
        #    for a in abs:
       #         x = x + 1

        #center = []
        #for x,y in enumerate(absolut_difference):
            #print(abs)
            #(x,y) = obj
            #cx = int((x + x + msg.width)/2)
            #cy = int((y + y + msg.height)/2)
            #center.append((cx,cy))

        
        
        #for point in center:
            #cv2.circle(absolut_difference,point,5,(0,0,255),-1)

        #cv2.imshow("contours",contours)
        cv2.imshow("change in foreground", cv2.resize(absolut_difference, (600,500)))
        cv2.imshow("processTheFrame", cv2.resize(frame, (600,500)))

        key = cv2.waitKey(80)
        if key == 27:
            break

    cv2.destroyAllWindows()

readBagFile()
ConvertTOFrame()
