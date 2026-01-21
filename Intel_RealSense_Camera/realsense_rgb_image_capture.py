import pyrealsense2 as rs
import numpy as np
import cv2


pipeline = rs.pipeline()


config = rs.config()
#bgr8 is for RGB images 
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 6)

profile = pipeline.start(config)

sensor = profile.get_device().query_sensors()[1]
sensor.set_option(rs.option.white_balance, 4000)
sensor.set_option(rs.option.exposure, 150)

try:

    frames = pipeline.wait_for_frames()

   
    color_frame = frames.get_color_frame()
    
 
    color_image = np.asanyarray(color_frame.get_data())
    
    
    cv2.imshow("Color Image", color_image)

    cv2.imwrite("color_image.png", color_image)


    cv2.waitKey(0)

finally:

    pipeline.stop()
    cv2.destroyAllWindows()
