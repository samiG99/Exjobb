import rospy
from sensor_msgs.msg import Image
import rosbag 
import cv2
import pubFrame
import numpy as np 
from sklearn.linear_model import RANSACRegressor
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
msg_cpy = None
frm = None
previus_frame = None
i =0 

def callback(msg):
    global msg_cpy
    #rospy.loginfo(msg)
    msg_cpy = msg
    #print("Image shape:", msg_cpy.height, msg_cpy.width)
    #print("Image data type:", msg_cpy.encoding)

def listener():
    global msg_cpy
    global i
    rospy.init_node('listener', anonymous=True)
    pup = rospy.Publisher("data",Image,queue_size=100)
    rospy.Subscriber('/robot/fisheye_front/image_color', Image, callback)
    rate = rospy.Rate(10) # 100Hz
    while not rospy.is_shutdown():
        key = cv2.waitKey(1)    
        if key == 27:
            exit()
        
        if msg_cpy is not None:
            frame = ConvertTOFrame(msg_cpy)
            dat = objectdetection(frame) 
            my_data = ConvertToType(dat)
            i = i +1
            pup.publish(my_data)
            print(i)
            msg_cpy = None
            
            #print(i)
        rate.sleep()
        #rospy.spin()


def ConvertTOFrame(msg):
    #convert the msg data to binary data, the string to binary, binary to bytearray
    BIN_DATA = bytearray(msg.data)
    
    #converting the msg to frame 
    frame = np.array(BIN_DATA, dtype=np.uint8)
    frame = frame.reshape(msg.height, msg.width, -1)
    #print(frame.shape)
    return frame


def ConvertToType(frame):
    bridge = CvBridge()
    # Convert the NumPy array to a ROS image message
    img_msg = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
    return img_msg


def objectdetection(frm):
    global previus_frame
    
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
    _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY) # if we have something less than 25 
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    pt1 = (935 , 690 )
    pt2 = (1080 , 780 )

    # Draw the box rectangle
    cv2.rectangle(absolut_difference, pt1, pt2, (0, 255, 0), 5)

    # looking if there is some object moving in the frame, detect the obejct
    if len(contours)>0:
    # Find the coordinates of all the non-zero pixels, that have value greater than 0 
        points = np.argwhere(thresh > 0)
        #we have 2D array with shape (n,2), x is the second colum, and y is the first one 
        x = points[:,1]
        y = points[:,0]

        if len(x) < 3:
            # if there are less than three points, we cannot fit a curve
            return

        
        #the algorithm will estimate a quadratic model if there are at least 3 point data
        model = RANSACRegressor(random_state=0, min_samples=3)
        #we are using fit to estamiate the coefficients of the quadratic curve. 
        #we will have a curve that present our object moviment therefore we should to have a parabolic trajectory
        model.fit(np.column_stack((x**2, x, np.ones_like(x))), y)
        #here we have a booelan over which data in inlier
        inlier_mask = model.inlier_mask_

        # Generate points along the curve for plotting
        #we generate a array of 100 equallyt spaced value btw min and max x
        curve_x = np.linspace(np.min(x), np.max(x), 100)
        #we are using predict to gererate the y coordinate
        curve_y = model.predict(np.column_stack((curve_x**2, curve_x, np.ones_like(curve_x))))

        # Get the center of mass of the inlier points as the point of the moving object, by using mean 
        center_of_mass = np.round(np.mean(np.column_stack((x[inlier_mask], y[inlier_mask])), axis=0)).astype(int)

        # Draw the curve and the center of mass on the image
        cv2.polylines(absolut_difference, [np.column_stack((curve_x, curve_y)).astype(np.int32)], False, (0, 255, 0), thickness=2)
        cv2.circle(absolut_difference, tuple(center_of_mass), 30, (0, 0, 255), -1)

        # Check if the center of mass is inside the box
        if center_of_mass[0] >= 935 and center_of_mass[0] <= 1080 and center_of_mass[1] >= 690 and center_of_mass[1] <= 780:
            print("Object is inside the box!")
            pass
        else:
            print("Object is outside the box.")
            pass

        # Create a mask for the moving object, isolate the moving object from the background
        #mask = np.zeros_like(absolut_difference1)
        #cv2.circle(mask, tuple(center_of_mass), 50, (255, 255, 255), -1)
        

    cv2.imshow("change in ", cv2.resize(absolut_difference, (800,600)))
    #cv2.waitKey(0)
    return absolut_difference

    #if mask is not None:
    #    cv2.imshow("mask ", cv2.resize(mask, (800,600)))

    #
    #cv2.imshow("change in foreground", cv2.resize(moving_object, (800,600)))
    #cv2.imshow("change in foreground", cv2.resize(frame, (800,600)))
    #cv2.imshow("processTheFrame", cv2.resize(frame, (600,500)))
    #key = cv2.waitKey(0)
    #if key == 27:
    #    exit()
    #cv2.imshow("change in foreground", cv2.resize(frm, (800,600)))
    #key = cv2.waitKey(1)
            
    #if key == 27:
    #    exit()


    
if __name__ == '__main__':
    listener()

