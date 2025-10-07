import cv2
import numpy as np

# import cv2.aruco as aruco

# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# img = aruco.drawMarker(aruco_dict, 2, 170)  # marker ID 0, size 1000px
# cv2.imwrite("Marker2.png", img)

cv_image = cv2.imread("img1.jpeg")
cv_image = cv2.resize(cv_image, (300, 200)) 
hsv_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

def nothing(x):
    pass
cv2.namedWindow("Color Adjustment")
cv2.createTrackbar("Lower_H","Color Adjustment",0,255,nothing)
cv2.createTrackbar("Lower_S","Color Adjustment",0,255,nothing)
cv2.createTrackbar("Lower_V","Color Adjustment",0,255,nothing)

cv2.createTrackbar("Higher_H","Color Adjustment",255,255,nothing)
cv2.createTrackbar("Higher_S","Color Adjustment",255,255,nothing)
cv2.createTrackbar("Higher_V","Color Adjustment",255,255,nothing)
while True:
    # get trackbar positions
    l_h = cv2.getTrackbarPos("Lower_H","Color Adjustment")
    l_s = cv2.getTrackbarPos("Lower_S","Color Adjustment")
    l_v = cv2.getTrackbarPos("Lower_V","Color Adjustment")
    u_h = cv2.getTrackbarPos("Higher_H","Color Adjustment")
    u_s = cv2.getTrackbarPos("Higher_S","Color Adjustment")
    u_v = cv2.getTrackbarPos("Higher_V","Color Adjustment")

    print(f"color ranges : lower : {l_h,l_s,l_v} higher : {u_h,u_s,u_v}")
    # blue color ranges in hsv
    lower_bound = np.array([l_h,l_s,l_v]) 
    upper_bound = np.array([u_h,u_s,u_v])

    # create a binary mask
    mask = cv2.inRange(hsv_img,lower_bound,upper_bound)

    # apply the mask to the original image
    blue_roi_img = cv2.bitwise_and(cv_image,cv_image,mask=mask)

    cv2.imshow("MASK",mask)
    cv2.imshow("hsv",hsv_img)
    cv2.imshow("blue_roi_img",blue_roi_img)
    key = cv2.waitKey(1)

    if key == 27:
        break
cv2.destroyAllWindows()