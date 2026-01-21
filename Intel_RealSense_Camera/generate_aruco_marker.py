import numpy as np
import cv2



aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)

marker_id = 0

marker_size = 700
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)


cv2.imwrite("aruco_marker_7x7_50.png", marker_image)

cv2.imshow('7x7 ArUco Marker', marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()