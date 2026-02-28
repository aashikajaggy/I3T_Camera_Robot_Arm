import pyrealsense2 as rs
import numpy as np
import cv2
import pandas as pd
import time

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
depth_scale = depth_sensor.get_depth_scale()


align_to = rs.stream.color
align = rs.align(align_to)

#this is intrinisc matrix for only the X and Y
def pixel_to_3d(u, v, depth_value, intrinsics):
    
    fx = intrinsics.fx #focal length in the x coordinate
    fy = intrinsics.fy #focal length in the y coordinate 
    ppx = intrinsics.ppx #principal point, where the optical axis intersects the image plane in the x axis
    ppy = intrinsics.ppy #principal point, where the optical axis intersects the image plane in the y axis 

    
    X = (u - ppx) * depth_value / fx #u=fx*x+cx, fy*y+cy
    Y = (v - ppy) * depth_value / fy
    Z = depth_value #z value represents the distance 

    return X, Y, Z

coordinate_list = []
last_capture_time = time.time()

try:

    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame() 
        color_frame = aligned_frames.get_color_frame()

        color_stream_profile = color_frame.get_profile()
        color_intrinsics = color_stream_profile.as_video_stream_profile().get_intrinsics()



        depth_image = np.asanyarray(aligned_depth_frame.get_data())
       

        color_image = np.asanyarray(color_frame.get_data())


        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
        parameters =  cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

        current_time = time.time()
        if current_time - last_capture_time >= 15:
            last_capture_time = current_time

            if corners is not None and len(corners) > 0:
                try:
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
                    print(depth_image[int(round(corner_points[0][1])),int(round(corner_points[1][1]))])
                    kernel = np.zeros((kernel_size, kernel_size), dtype=np.uint16)

                    for i in range(kernel_size):
                        for j in range(kernel_size):
                            x = center_x + (i - half_kernel)
                            y = center_y + (j - half_kernel)
                            if 0 <= x < width and 0 <= y < height:
                                kernel[i, j] = depth_image[y, x]

                    print("kernel")
                    print(kernel)

                    non_zero_indices = np.nonzero(kernel)
                    non_zero_values = kernel[non_zero_indices]

                    if len(non_zero_values) > 0:
                        average_depth = np.mean(non_zero_values[0])
                        print("average_depth")
                        print(average_depth)

                        X, Y, Z = pixel_to_3d(center_x, center_y, average_depth, color_intrinsics)

                        print("X")
                        print(X)
                        print("Y")
                        print(Y)

                        coordinate_list.append([X, Y, Z])
                    else:
                        print("No non-zero depth values in kernel, skipping.")

                except Exception as e:
                    print("Error processing marker:", e)
            else:
                print("No marker detected, skipping capture.")

        cv2.imshow("Color Stream", color_image)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
      
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    if coordinate_list:
        df = pd.DataFrame(coordinate_list, columns=["X", "Y", "Z"])
        df.to_csv("coordinates.csv", index=False)
        print("Saved captured coordinates to coordinates.csv")
    else:
        print("No coordinates captured.")