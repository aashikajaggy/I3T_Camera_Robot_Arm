import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyrealsense2 as rs
import cv2
import numpy as np

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
        
        
        #we are publishing to the mrker location topic with 10 outgoing messages stored inside of a queue
        self.publisher_ = self.create_publisher(String, 'marker_location', 10)
        time_period = 0.5
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.i = 0

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

        #converting into numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
    
        return depth_image, color_image, color_intrinsics
    
        
    def capture_marker_depth(self, depth_image, color_image):
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)

        corner_points = corners[0][0]

        avg_x=(corner_points[1][0]+corner_points[2][0])/2
        avg_x1=(corner_points[0][0]+corner_points[3][0])/2

        avg_y = (corner_points[0][1]+corner_points[1][1])/2
        avg_y1 = (corner_points[2][1]+corner_points[3][1])/2

        middle_x = (avg_x+avg_x1)/2
        middle_y = (avg_y+avg_y1)/2

        center_x = int(round(middle_x))
        center_y = int(round(middle_y))

        kernel_size = 50
        half_kernel = kernel_size // 2

        height, width = depth_image.shape

        kernel = np.zeros((kernel_size, kernel_size), dtype=np.uint16)

        for i in range(kernel_size):
            for j in range(kernel_size):
                x = center_x + (i - half_kernel)
                y = center_y + (j - half_kernel)
                if 0 <= x < width and 0 <= y < height:
                    kernel[i, j] = depth_image[y, x]
        non_zero_indices = np.nonzero(kernel)

        # Extract non-zero values using the indices
        non_zero_values = kernel[non_zero_indices]
        average_depth = np.mean(non_zero_values[0])

        return center_x, center_y, average_depth
    
    def pixel_to_3d(self, u, v, depth_value, intrinsics):
    
        fx = intrinsics.fx #focal length in the x coordinate
        fy = intrinsics.fy #focal length in the y coordinate 
        ppx = intrinsics.ppx #principal point, where the optical axis intersects the image plane in the x axis
        ppy = intrinsics.ppy #principal point, where the optical axis intersects the image plane in the y axis 

        
        X = (u - ppx) * depth_value / fx #u=fx*x+cx, fy*y+cy
        Y = (v - ppy) * depth_value / fy
        Z = depth_value #z value represents the distance 

        return X, Y, Z
    
    def timer_callback(self):
        depth_image, color_image, color_intrinsics = self.capture_depth_color_image()
        center_x, center_y, average_depth = self.capture_marker_depth(depth_image, color_image)
        X, Y, Z = self.pixel_to_3d(center_x, center_y, average_depth, color_intrinsics)
        msg = String()
        msg.data = f'counter: {self.i}, X: {X}, Y:{Y}, Z:{Z}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


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

