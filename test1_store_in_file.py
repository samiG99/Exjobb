import rosbag 
import cv2
import numpy as np 

arr = []
counter = 0
def readBagFile():
    readingBagFile = rosbag.Bag("2022-08-26-15-18-39.bag")
    for topic, msg, t in readingBagFile.read_messages(topics=['/robot/k4a_top/depth/image_rect']):
        arr.append(msg)
        #print("Message",msg)
    
    if len(arr)<10:
        raise ValueError ("the size of array is not enought")

    readingBagFile.close()

def ConvertTOFrame():
    global counter
    with open("D.txt", "w") as f:
        for msg in arr:
            #convert the msg data to binary data, the string to binary, binary to bytearray  
            BIN_DATA = bytearray(msg.data)
            
            #converting the msg to frame 
            frame = np.array(BIN_DATA, dtype= np.uint8)
            frame = frame.reshape(msg.height, msg.width,-1) #we tell the numpy to compute the size of thrid dimension based of the size of the array
            for i in range(frame.shape[0]):
                for j in range(frame.shape[1]):
                    #if (frame[i][j][3] != 64 and frame[i][j][3] != 127 and frame[i][j][3] != 63):
                    print(f"Frame ID: {counter} pixel {i}x{j}: B:{frame[i][j][0]}  G:{frame[i][j][1]}  R: {frame[i][j][2]} D: {frame[i][j][3]}", file=f, end="")
                    f.write("\n")
            #printFrame_c(frame)
            print(f"frame ID: {counter}", frame.shape)
            #use opencv to process the frame
            processTheFrame = frame
            cv2.imshow("processTheFrame" , cv2.resize(processTheFrame, (800,600)))
            key = cv2.waitKey(80)
            counter +=1
            if key ==27:
                break
        cv2.destroyAllWindows()
        counter = 0
        f.close()

def printFrame_c(frame):   #for color
    for i in range(frame.shape[0]):
        for j in range(frame.shape[1]):
                print(f"pixel {i}x{j}: B:{frame[i][j][0]}  G:{frame[i][j][1]}  R: {frame[i][j][2]} ")

#def printFrame_r(frame):   #for image rect
#    global counter

readBagFile()  
ConvertTOFrame()
