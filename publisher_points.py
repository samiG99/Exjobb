import rospy
import rosbag
from sensor_msgs.msg import PointCloud2

bagfile39 = '2022-08-26-15-18-39.bag'
bagfile17 = '2022-08-26-15-20-17.bag'

image_color = '/robot/fisheye_front/image_color'
image_rect = '/robot/k4a_top/depth/image_rect'
points = '/robot/k4a_top/depth/points'
index = 0

def talker():
    global index
    index = 0
    pub = rospy.Publisher(points, PointCloud2, queue_size=1000000)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(3) # 5hz
    bag = rosbag.Bag(bagfile39)
    for topic, msg, t in bag.read_messages(topics=[points]):
        #rospy.loginfo(msg)
        pub.publish(msg)
        index = index+1
        rate.sleep()
    bag.close()
    index = 0

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
