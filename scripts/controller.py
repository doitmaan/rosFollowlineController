#!/usr/bin/env python
import rospy
from std_msgs.msg import String
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I got %s", data.data)

def ControllerSubscriber():    
    rospy.init_node('ControllerSubscriber', anonymous=True)
    rospy.Subscriber("VisionState", String, callback)


    rospy.spin()

class SendSerialThread(threading.Thread):

    def __init__(self, String):
        threading.Thread.__init__(self)
        self.String = String

       
        # self.image_pub = rospy.Publisher("/line_image/image_raw", Image, queue_size=1)
        # self.maskImage_pub = rospy.Publisher("/line_image/mask_raw", Image, queue_size=1)
        # self.image_pub = rospy.Publisher("/line_image/image_raw/compressed", CompressedImage, queue_size=1)


if __name__ == '__main__':
                ControllerSubscriber()

