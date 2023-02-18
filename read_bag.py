import rosbag 
import cv2
import numpy as np 

arr = []
def readBagFile():
    readingBagFile = rosbag.Bag("2022-08-26-15-20-17.bag")
    for topic, msg, t in readingBagFile.read_messages(topics=['/robot/fisheye_front/image_color']):
        arr.append(msg)
        #print("Message",msg)
    
    if len(arr)<10:
        raise ValueError ("teh size of array is not enought")

    readingBagFile.close()

def ConvertTOFrame():
    for msg in arr[20:]:
        #convert the msg data to binary data, the string to binary, binary to bytearray  
        BIN_DATA = bytearray(msg.data)
        
        #converting the msg to frame 
        frame = np.array(BIN_DATA, dtype= np.uint8)
        frame = frame.reshape(msg.height, msg.width,-1) #we tell the numpy to compute the size of thrid dimension based of the size of the array
        #print(frame)
        #print(np.max(frame))
        processTheFrame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        cv2.imshow("processTheFrame" ,processTheFrame)
        key = cv2.waitKey(80)
        if key ==27:
            break
    cv2.destroyAllWindows()



readBagFile()
ConvertTOFrame()
