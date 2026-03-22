import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped

import tf2_ros
from tf_transformations import quaternion_from_matrix



class  MarkerFinderPublisher(Node):
    def __init__(self):
        #initialize the node
        super().__init__('marker_finder_publisher')

        self.subscription_aruco = self.create_subscription(
            Image,
            'camera/camera/color/image_raw', 
            self.publish_tf_frame,
            10)
        self.subscription_cam_info = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.capture_camera_info,
            10
        )
        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        

    #code to capture the depth and color image numpy arrays along with the intrinsic matrix for the color image
    def capture_camera_info(self, msg: CameraInfo):
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape(3, 3)
            D = np.array(msg.d)
            self.camera_matrix = K
            self.dist_coeffs = D
            self.get_logger().info(f"Camera matrix set:\n{K}\nDistortion:\n{D}")
    
        
    def publish_tf_frame(self, msg: Image):

        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
        marker_length = 0.08
        if ids is None:
            return
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            marker_length,
            self.camera_matrix,
            self.dist_coeffs
        )
        if corners is None:
            self.get_logger().warn("no corners detected!")
            return
        for i in range(len(ids)):
            rvec = rvecs[i]
            tvec = tvecs[i]
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec.flatten()

            quat = quaternion_from_matrix(T)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera_color_optical_frame"
            t.child_frame_id = "marker"

            t.transform.translation.x = float(tvec[0][0])
            t.transform.translation.y = float(tvec[0][1])
            t.transform.translation.z = float(tvec[0][2])

            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    marker_finder_publisher = MarkerFinderPublisher()

    try:
        rclpy.spin(marker_finder_publisher)
    except KeyboardInterrupt:
        pass
    marker_finder_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


        








