import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np

from geometry_msgs.msg import TransformStamped

import tf2_ros
from tf_transformations import quaternion_from_matrix



class  MarkerFinderPublisher(Node):
    def __init__(self):
        #initialize the node
        super().__init__('marker_finder_publisher')
        #main streaming interface for the camera 
        self.pipeline = rs.pipeline()
        self.config = rs.config()


        #enables the depth stream 
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #enables the color stream
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self._start_pipeline()

        #timeout counter
        self.max_poll_ms = 500
        self.restart_after_timeouts = 5
        self.poll_tries = 6            # total ~3s per timer tick

        #sets u; frame alignment to map the color image to the depth image 
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.consecutive_timeouts = 0

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        
        #we are publishing to the mrker location topic with 10 outgoing messages stored inside of a queue
        time_period = 0.5
        self.timer = self.create_timer(time_period, self.timer_callback)

    def _poll_frames(self):
        # Try poll loop to avoid long blocking timeouts
        for _ in range(self.poll_tries):
            try:
                fs = self.pipeline.wait_for_frames(self.max_poll_ms)
            except Exception:
                fs = None
            if fs:
                return fs
        return None

    def _start_pipeline(self):
        # (Re)start pipeline and related helpers
        self.profile = self.pipeline.start(self.config)

        # Reduce internal frame queues to avoid stale frames
        dev = self.profile.get_device()
        for s in dev.sensors:
            if s.supports(rs.option.frames_queue_size):
                try:
                    s.set_option(rs.option.frames_queue_size, 1)
                except Exception:
                    pass

        self.depth_sensor = dev.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)

        # Warmup (non-fatal if some time out)
        for _ in range(10):
            try:
                fs = self.pipeline.wait_for_frames(200)
                if fs: break
            except Exception:
                pass
        self.get_logger().info(f"Depth scale: {self.depth_scale:.6f}. Pipeline started.")

    def _stop_pipeline(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
    
    
    def _restart_pipeline(self, do_hw_reset=False):
        self.get_logger().warn("Restarting RealSense pipeline...")
        self._stop_pipeline()
        if do_hw_reset:
            try:
                self.profile.get_device().hardware_reset()
            except Exception:
                pass
        self._start_pipeline()
        self.consecutive_timeouts = 0
    
    #code to capture the depth and color image numpy arrays along with the intrinsic matrix for the color image
    def capture_depth_color_image(self):
        
        #we need to eventually implement constant streaming, updating values in the topic 
        frames = self._poll_frames()

        #handling failures to receive frames from a camera pipeline 
        if frames is None:
            self.consecutive_timeouts += 1
            self.get_logger().warn(f"No frames ({self.consecutive_timeouts} consecutive timeouts).")
            if self.consecutive_timeouts >= self.restart_after_timeouts:
                # First soft restart, then hard reset if still bad
                hard = (self.consecutive_timeouts >= 2*self.restart_after_timeouts)
                self._restart_pipeline(do_hw_reset=hard)
            return

        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame() 
        color_frame = aligned_frames.get_color_frame()

        color_stream_profile = color_frame.get_profile()
        color_intrinsics = color_stream_profile.as_video_stream_profile().get_intrinsics()

        fx = color_intrinsics.fx
        fy = color_intrinsics.fy
        cx = color_intrinsics.ppx
        cy = color_intrinsics.ppy
        dist_coeffs = np.array(color_intrinsics.coeffs)

        color_image = np.asanyarray(color_frame.get_data())


        intrinsic_camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)

        self.get_logger().info(f"Camera matrix:\n{intrinsic_camera_matrix}")
        self.get_logger().info(f"Distortion coeffs: {dist_coeffs}")

        return color_image, intrinsic_camera_matrix, dist_coeffs
    
        
    def capture_marker_rvec_tvec(self, color_image, intrinsic_camera_matrix, dist_coeffs):
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
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
            intrinsic_camera_matrix,
            dist_coeffs
        )
        return rvecs, tvecs, ids

      
    def timer_callback(self):
        result = self.capture_depth_color_image()
        if result is None:
            self.get_logger().warn("NOT DETECTING MARKER!!")
            return
        color_image, intrinsic_camera_matrix, dist_coeffs = result
        result1 = self.capture_marker_rvec_tvec(color_image, intrinsic_camera_matrix, dist_coeffs)
        if result1 is None:
            self.get_logger().warn("RVEC AND TVEC ISSUE")
            return
        rvecs, tvecs, ids = result1
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
            t.header.frame_id = "camera_color_frame"
            t.child_frame_id = f"marker_{ids[i][0]}"

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


        








