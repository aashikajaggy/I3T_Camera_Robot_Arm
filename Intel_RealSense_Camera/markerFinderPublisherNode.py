import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyrealsense2 as rs
import cv2
import numpy as np

pipeline = rs.pipeline()


config = rs.config()


pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)


config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)



profile = pipeline.start(config)


depth_sensor = profile.get_device().first_depth_sensor()

align_to = rs.stream.color
align = rs.align(align_to)

class  MarkerFinderPublisher(Node):
    def __init__(self):
        super().__init__('marker_finder_publisher')
        self.publisher_ = self.create_publisher(String, 'marker_location', 10)
        time_period = 0.5
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.i = 0

    #code to capture the depth and color image numpy arrays along with the intrinsic matrix for the color image

    def capture_depth_color_image(self):
        try:
            #we need to eventually implement constant streaming, updating values in the topic 
            frames = pipeline.wait_for_frames()

            aligned_frames = align.process(frames)


            aligned_depth_frame = aligned_frames.get_depth_frame() 
            color_frame = aligned_frames.get_color_frame()

            color_stream_profile = color_frame.get_profile()
            color_intrinsics = color_stream_profile.as_video_stream_profile().get_intrinsics()

            depth_image = np.asanyarray(aligned_depth_frame.get_data())

            


            color_image = np.asanyarray(color_frame.get_data())
            key = cv2.waitKey(1)

            return depth_image, color_image, color_intrinsics

        finally:
                pipeline.stop()
        
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

    rclpy.spin(marker_finder_publisher)
    marker_finder_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
