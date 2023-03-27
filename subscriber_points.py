import rospy
import publisher_points as publ
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import cv2
import numpy as np 
import struct
import sensor_msgs.point_cloud2 as pc2

frm = None
previus_frame = None
msg_cpy = None


def callback(msg):
    ""
    global msg_cpy
    msg_cpy = msg
    #fields = msg_cpy.fields
    #print(fields)
    #rospy.loginfo(msg)
    #print("Image shape:", msg_cpy.height, msg_cpy.width)

def listener():
    global msg_cpy    
    message_processed = True
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('datapoints', PointCloud2, queue_size=100)
    rospy.Subscriber(publ.points, PointCloud2, callback)
    # Create publisher object to publish points to RViz
    rate = rospy.Rate(3) # 100Hz
    BG = None
    while not rospy.is_shutdown():
        key = cv2.waitKey(1)
        if key == 27:
            exit()
        if msg_cpy is not None and message_processed:
            point_cloud = pc2.read_points(msg_cpy, skip_nans=False, field_names= ("x", "y", "z"))
            points = np.array(list(point_cloud))
            x = points[:,0]
            y = points[:,1]
            z = points[:,2]
            #print(f"X: {x} with size of {len(x)}")
            #print(f"Y: {y} with size of {len(y)}")
            #print(f"Z: {z} with size of {len(z)}")
            publ.index = publ.index+1
            if (publ.index == 1):
                BG = msg_cpy
            
            elif (publ.index >= 41):
                msg_cpy = None
                publ.index = 0
                continue
            
            else:
                for j in range(len(points[:,0])):
                    xi = x[j]
                    yi = y[j]
                    zi = z[j]
                    #print(f"point {i}: X:{xi}\tY:{yi}\tZ:{zi}")
            print(publ.index)
            pub.publish(msg_cpy)
            FM = msg_cpy

            if FM == BG:
                print("yes")
        elif msg_cpy is None:
            message_processed = True
            
        rate.sleep()    
    

if __name__ == '__main__':
    try:
        listener()

    except rospy.ROSInterruptException:
        pass



"""
"save" the first PointCloud as "Background"
for all other pointclouds
    iterate over all the points
    for each point (where p=x,y,z)
        find closest point q where q is a subset of "background"
        if |q-p| > d where d is the threshold
            add p to O (O is a new pointcloud)
"""
