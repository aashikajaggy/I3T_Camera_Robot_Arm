import pyrealsense2 as rs
import numpy as np


pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)  



pipeline.start(config)


depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
depth_intrinsics = depth_sensor.get_stream_profiles()[0].as_video_stream_profile().get_intrinsics()


print("Intrinsic matrix:")
print("Focal lengths: ", depth_intrinsics.fx, depth_intrinsics.fy)
print("Principal point: ", depth_intrinsics.ppx, depth_intrinsics.ppy)

def pixel_to_3d(u, v, depth_value, intrinsics):
    
    fx = intrinsics.fx #focal length in the x coordinate
    fy = intrinsics.fy #focal length in the y coordinate 
    ppx = intrinsics.ppx #principal point, where the optical axis intersects the image plane in the x axis
    ppy = intrinsics.ppy #principal point, where the optical axis intersects the image plane in the y axis 

    
    X = (u - ppx) * depth_value / fx #u=fx*x+cx, fy*y+cy
    Y = (v - ppy) * depth_value / fy
    Z = depth_value #z value represents the distance 

    return X, Y, Z

try:
    
    while True:    
        frames = pipeline.wait_for_frames()
            
            
        depth_frame = frames.get_depth_frame()
            
            
        u, v = 330, 250
        depth_value = depth_frame.get_distance(u, v)  

        
        X, Y, Z = pixel_to_3d(u, v, depth_value, depth_intrinsics)
            
        print(f"3D Coordinates at pixel ({u}, {v}):")
        print(f"X: {X:.3f} m, Y: {Y:.3f} m, Z: {Z:.3f} m")

finally:
    print("Stopping the pipeline.")
    pipeline.stop()

