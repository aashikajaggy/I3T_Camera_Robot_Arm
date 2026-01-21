import pyrealsense2 as rs
import numpy as np
import cv2


pipeline = rs.pipeline()


config = rs.config()
#bgr8 is for RGB images 
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)

pipeline.start(config)


try:

    frames = pipeline.wait_for_frames()

   
    depth_frame = frames.get_depth_frame()
    
 
    depth_image = np.asanyarray(depth_frame.get_data())

    nonzero_values = np.nonzero(depth_image)
    print(depth_image[nonzero_values])


    
    
    cv2.imshow("Depth Image", depth_image)

    cv2.imwrite("depth_image.png",depth_image)



    cv2.waitKey(0)

finally:

    pipeline.stop()
    cv2.destroyAllWindows()
