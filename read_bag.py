import rosbag 
import cv2
import numpy as np 

arr = []
def readBagFile():
    readingBagFile = rosbag.Bag("2022-08-26-15-18-39.bag")
    for topic, msg, t in readingBagFile.read_messages(topics=['/robot/k4a_top/depth/image_rect']):
        arr.append(msg)
        #print("Message",msg)

    readingBagFile.close()

def ConvertTOFrame():
    for msg in arr:
        #convert the msg data to binary data, the string to binary, binary to bytearray  
        BIN_DATA = bytearray(msg.data)
        
        #converting the msg to frame 
        frame = np.array(BIN_DATA, dtype= np.uint16)
        frame = frame.reshape(msg.height, msg.width, msg.step)
        #frame = frame.reshape(*frame)
        #use opencv to process the frame
        processTheFrame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        cv2.imshow("processTheFrame" ,processTheFrame)
        key = cv2.waitKey(80)
        if key ==27:
            break
    cv2.destroyAllWindows()



readBagFile()
ConvertTOFrame()
