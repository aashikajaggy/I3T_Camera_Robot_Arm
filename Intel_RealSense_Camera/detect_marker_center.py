import pyrealsense2 as rs
import numpy as np
import cv2


ss_image = "./color_image.png"

read_img = cv2.imread(ss_image)


print(read_img.shape)


if len(read_img.shape) == 2: 
    image = cv2.cvtColor(read_img, cv2.COLOR_GRAY2BGR)
elif read_img.shape[2] == 3:  
    image = cv2.cvtColor(read_img, cv2.COLOR_RGBA2BGR)


gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
corners, ids, rejected = detector.detectMarkers(gray)

print("corners", corners)

depth_image = cv2.imread("depth_image.png")



aligner = cv2.rgbd.RegisterDepth()
aligned_depth = aligner.registerDepth(read_img, depth_image)

'''


middle_x = 751.75
middle_y = 355.25

center_x = int(round(middle_x))
center_y = int(round(middle_y))

kernel_size = 5
half_kernel = kernel_size // 2

height, width = depth_image.shape
kernel = np.zeros((kernel_size, kernel_size), dtype=np.uint16)

for i in range(kernel_size):
    for j in range(kernel_size):
        x = center_x + (i - half_kernel)
        y = center_y + (j - half_kernel)
        if 0 <= x < width and 0 <= y < height:
            kernel[i, j] = depth_image[y, x]
'''




