import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import os
import tf
import numpy as np  
from threading import Lock


bridge = CvBridge()


os.makedirs("images", exist_ok=True)
os.makedirs("camera_params", exist_ok=True)

lock = Lock()


camera_data = {}

def image_callback(camera_name, msg):
    with lock:
        if camera_name not in camera_data:
            camera_data[camera_name] = {}
        camera_data[camera_name]['image'] = msg

def camera_info_callback(camera_name, msg):
    with lock:
        if camera_name not in camera_data:
            camera_data[camera_name] = {}
        camera_data[camera_name]['camera_info'] = msg

def save_all_cameras(listener, cameras,R_align):
    rate = rospy.Rate(1)  
    while not rospy.is_shutdown():
        with lock:
            for cam in cameras:
                name = cam["name"]
                if name in camera_data and 'image' in camera_data[name] and 'camera_info' in camera_data[name]:
                    image_msg = camera_data[name]['image']
                    camera_info_msg = camera_data[name]['camera_info']
                    

                    extrinsic_params = get_camera_extrinsics(listener, cam["frame"],R_align)
                    

                    if extrinsic_params:
                        save_image_and_params(image_msg, camera_info_msg, name, extrinsic_params)
                    

                    del camera_data[name]
        rate.sleep()

def get_camera_extrinsics(listener, camera_frame, R_align):
    try:

        (trans, rot) = listener.lookupTransform('/world', camera_frame, rospy.Time(0))
        rospy.loginfo(f"Camera: {camera_frame}, Translation: {trans}, Rotation (quaternion): {rot}")
        print(rot)
        

        R_current = tf.transformations.quaternion_matrix(rot)[:3, :3]
        

        R_current_inv = np.linalg.inv(R_current)  
        

        R_world = R_align @ R_current_inv# note that here the R current is described as the rotation along Z in world
        

        det = np.linalg.det(R_world)
        if not np.isclose(det, 1.0):
            rospy.logwarn(f"R_world forbidden {det} include more than rotation")
        
        extrinsic_params = {
            'translation': list(trans),
            'rotation': R_world.tolist()
        }
        
        return extrinsic_params
    
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Failed to get extrinsics for {camera_frame}: {e}")
        return None


def save_image_and_params(image_msg, camera_info_msg, camera_name, extrinsic_params):
    try:

        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        

        image_path = f"images/{camera_name}.png"
        cv2.imwrite(image_path, cv_image)
        rospy.loginfo(f"Saved image to {image_path}")
        

        camera_params = {
            "width": camera_info_msg.width,
            "height": camera_info_msg.height,
            "fx": camera_info_msg.K[0],  # fx
            "fy": camera_info_msg.K[4],  # fy
            "cx": camera_info_msg.K[2],  # cx
            "cy": camera_info_msg.K[5],  # cy
            "distortion_model": camera_info_msg.distortion_model,
            "D": camera_info_msg.D,  
            "rotation_matrix": extrinsic_params['rotation'],
            "translation_vector": extrinsic_params['translation'],  
            "image_name": os.path.basename(image_path)
        }
        

        params_path = f"camera_params/{camera_name}_params.yaml"
        with open(params_path, 'w') as file:
            yaml.dump(camera_params, file)
        
        rospy.loginfo(f"Saved camera parameters to {params_path}")
        
    except Exception as e:
        rospy.logerr(f"Error saving image or parameters: {str(e)}")

def main():
    rospy.init_node('camera_saver', anonymous=True)
    

    listener = tf.TransformListener()
    rospy.loginfo("Waiting for TF cache to load...")
    rospy.sleep(2)  
    cameras = [
        {"name": "camera_1", "image_topic": "/camera_1/raw_image", "info_topic": "/camera_1/camera_info", "frame": "camera_link_1"},
        {"name": "camera_2", "image_topic": "/camera_2/raw_image", "info_topic": "/camera_2/camera_info", "frame": "camera_link_2"},
        {"name": "camera_3", "image_topic": "/camera_3/raw_image", "info_topic": "/camera_3/camera_info", "frame": "camera_link_3"},
        {"name": "camera_4", "image_topic": "/camera_4/raw_image", "info_topic": "/camera_4/camera_info", "frame": "camera_link_4"},
        {"name": "camera_5", "image_topic": "/camera_5/raw_image", "info_topic": "/camera_5/camera_info", "frame": "camera_link_5"},
        {"name": "camera_6", "image_topic": "/camera_6/raw_image", "info_topic": "/camera_6/camera_info", "frame": "camera_link_6"},
        {"name": "camera_7", "image_topic": "/camera_7/raw_image", "info_topic": "/camera_7/camera_info", "frame": "camera_link_7"},
        {"name": "camera_8", "image_topic": "/camera_8/raw_image", "info_topic": "/camera_8/camera_info", "frame": "camera_link_8"},
        {"name": "camera_top", "image_topic": "/camera_top/raw_image", "info_topic": "/camera_top/camera_info", "frame": "camera_link_top"},
    ]
    R_align = np.array([
        [0, -1, 0],
        [0, 0, -1],
        [1, 0, 0]
    ])

    for cam in cameras:
        rospy.Subscriber(cam["image_topic"], Image, lambda msg, name=cam["name"]: image_callback(name, msg))
        rospy.Subscriber(cam["info_topic"], CameraInfo, lambda msg, name=cam["name"]: camera_info_callback(name, msg))
        rospy.loginfo(f"订阅 {cam['name']} 的话题: {cam['image_topic']} 和 {cam['info_topic']}")
    

    save_all_cameras(listener, cameras, R_align)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

