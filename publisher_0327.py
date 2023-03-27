import rospy
import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import Image

bagfile39 = '2022-08-26-15-18-39.bag'
bagfile17 = '2022-08-26-15-20-17.bag'

image_color = '/robot/fisheye_front/image_color'
image_rect = '/robot/k4a_top/depth/image_rect'
points = '/robot/k4a_top/depth/points'

def talker():
    pub = rospy.Publisher(image_color, Image, queue_size=1000000)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(4) # 10hz
    bag = rosbag.Bag(bagfile17)
    for topic, msg, t in bag.read_messages(topics=[image_color]):
        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
    bag.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
