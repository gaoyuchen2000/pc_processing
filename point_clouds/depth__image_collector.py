import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os


bridge = CvBridge()


output_dir = "depth_images"
os.makedirs(output_dir, exist_ok=True)


def save_depth_image(camera_id, msg):
    try:

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        filename = os.path.join(output_dir, f"camera{camera_id}_depth.npy")

        np.save(filename, cv_image)
        rospy.loginfo(f"Saved depth image from camera {camera_id} to {filename}")
    except Exception as e:
        rospy.logerr(f"Failed to process depth image from camera {camera_id}: {e}")

if __name__ == "__main__":
    rospy.init_node("depth_image_saver")
    

    rospy.Subscriber("/camera1/depth/image_raw", Image, lambda msg: save_depth_image(1, msg))
    rospy.Subscriber("/camera2/depth/image_raw", Image, lambda msg: save_depth_image(2, msg))
    rospy.Subscriber("/camera3/depth/image_raw", Image, lambda msg: save_depth_image(3, msg))
    rospy.Subscriber("/camera4/depth/image_raw", Image, lambda msg: save_depth_image(4, msg))
    
    rospy.loginfo("Subscribed to depth image topics. Waiting for data...")
    rospy.spin()

