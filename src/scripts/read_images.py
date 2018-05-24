from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# YOLO imports.
import os
sys.path.append("/home/sniyaz/my-workspace/src/darknet/python")
import darknet

import pdb

class image_reader:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("EyeRecTooImage",Image,self.callback)

    # Load Darknet for YOLO.
    os.chdir("/home/sniyaz/my-workspace/src/darknet")
    self.net = darknet.load_net("/home/sniyaz/my-workspace/src/darknet/cfg/yolov3-tiny.cfg", "/home/sniyaz/my-workspace/src/darknet/weights/yolov3-tiny.weights", 0)
    self.meta = darknet.load_meta("/home/sniyaz/my-workspace/src/darknet/cfg/coco.data")

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Save to temp file so we can convert to YOLO format.
    scratch_path = "/home/sniyaz/my-workspace/src/darknet/temp.jpg"
    cv2.imwrite(scratch_path, cv_image)
    all_detections = darknet.detect(self.net, self.meta, scratch_path)

    # Draw the detections for the user to see.
    for detection in all_detections:
        center_x = detection[2][0]
        center_y = detection[2][1]

        bb_width = detection[2][2]
        bb_hieght = detection[2][3]

        bb_upper_left = (int(center_x - (bb_width/2)), int(center_y - (bb_hieght/2)))
        bb_lower_right = (int(center_x + (bb_width/2)), int(center_y + (bb_hieght/2)))

        cv2.rectangle(cv_image, bb_upper_left, bb_lower_right, (0,0,255), 2)



    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def main(args):
  ir = image_reader()
  rospy.init_node('image_reader', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
