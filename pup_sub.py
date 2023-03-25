import rospy
from sensor_msgs.msg import Image
import rosbag 
import cv2
import numpy as np 

msg_cpy = None
frm = None
previus_frame = None



def callback(msg):
    global msg_cpy
    #rospy.loginfo(msg)
    msg_cpy = msg
    print("Image shape:", msg_cpy.height, msg_cpy.width)
    print("Image data type:", msg_cpy.encoding)

def listener():
    global msg_cpy
    timer = 0
    rospy.init_node('listener', anonymous=True)
    
    while True:
        rospy.Subscriber('/robot/fisheye_front/image_color', Image, callback)
        if msg_cpy is not None:
            frm = ConvertTOFrame(msg_cpy)
            objectdetection(frm) 
        else:
            timer = timer + 1
        rospy.sleep(0.1)
        if timer > 100:
            cv2.destroyAllWindows()
            timer = 0


def ConvertTOFrame(msg_cpy):
    #convert the msg data to binary data, the string to binary, binary to bytearray
    BIN_DATA = bytearray(msg_cpy.data)
    
    #converting the msg to frame 
    frame = np.array(BIN_DATA, dtype=np.uint8)
    frame = frame.reshape(msg_cpy.height, msg_cpy.width, -1)
    #print(frame.shape)
    return frame



def objectdetection(frm):
    global previus_frame

    #previus_frame = None
    #for msg in arr:
        #frame = ConvertTOFrame(msg)
    if previus_frame is None:
        previus_frame = np.float32(frm)

    # Update the previous frame
    previus_frame = cv2.accumulateWeighted(frm, previus_frame, 0.3)

    frame_without_box = frm.copy()
    frame_without_box[690:785, 934:1085] = 0
    # Calculate the absolute difference between the current frame and the previous frame
    absolut_difference = cv2.absdiff(frame_without_box, cv2.convertScaleAbs(previus_frame))
    absolut_difference1 = cv2.absdiff(frm, cv2.convertScaleAbs(previus_frame))

    # Find contours of the moving object
    gray = cv2.cvtColor(absolut_difference1, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY) # if we have something less than 25 
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #box_size = [(1000,720), (1000, 720), (1000, 720), (1000, 720)]
    #center = (int((box_size[0][0] + box_size[1][0]) / 2), int((box_size[0][1] + box_size[2][1]) / 2))
    pt1 = (935 , 690 )
    pt2 = (1080 , 780 )

    # Draw the box rectangle
    cv2.rectangle(absolut_difference, pt1, pt2, (0, 255, 0), 5)

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
            cv2.circle(absolut_difference,  (cx, cy), 30, (0, 0, 255), -1)
            # Create a mask for the moving object
            mask = np.zeros_like(absolut_difference1)
            cv2.drawContours(mask, [largest_contour], -1, (255, 255, 255), -1)
            # Keep only the moving object in the difference image
            #moving_object = cv2.bitwise_and(absolut_difference, absolut_difference, mask=mask)


    else:
        moving_object = absolut_difference


    cv2.imshow("change in ", cv2.resize(absolut_difference, (800,600)))
    #cv2.imshow("change in foreground", cv2.resize(moving_object, (800,600)))
    #cv2.imshow("change in foreground", cv2.resize(frame, (800,600)))
    #cv2.imshow("processTheFrame", cv2.resize(frame, (600,500)))
    #key = cv2.waitKey(0)
    #if key == 27:
    #    exit()
    #cv2.imshow("change in foreground", cv2.resize(frm, (800,600)))
    key = cv2.waitKey(1)
            
    if key == 27:
        exit()


    
if __name__ == '__main__':
    try:
        listener()

    except rospy.ROSInterruptException:
        pass
