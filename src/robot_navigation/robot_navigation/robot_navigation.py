import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image , CameraInfo
from geometry_msgs.msg import TwistStamped
import cv2
from cv_bridge import CvBridge 
import numpy as np 
from constants.constants import aruco_distance_threshold
# linear speed
LINEAR_SPEED = 0.5 

# angular speed 
# Controller gains
KP = 0.001 # Proportional gain
KI = 0.0 # Integral gain
KD = 0.0  # Derivative gain

# KP = 0.0003  # Proportional gain
# KI = 0.0001 # Integral gain
# KD = 0.008  # Derivative gain

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.pub_ = self.create_publisher(
            TwistStamped,
            "/robot_diff_drive_controller/cmd_vel",
            10)
        self.camera_image_sub_ = self.create_subscription(
            Image,
            "/camera_sensor/image_raw",
            self.image_callback,
            10)
        self.camera_info_sub_ = self.create_subscription(CameraInfo,"/camera_sensor/camera_info",self.camera_info_callback,10)

        self.camera_image_sub_
        self.camera_info_sub_
        self.bridge_ = CvBridge()
        self.camera_matrix = None 
        self.dist_coeffs = None 
        self.marker_length = 1.0
        self.aruco_distance = None
        self.marker_id = None

        # PID controller variables
        self.previous_error = 0.0
        self.integral_error = 0.0

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def aruco_detection(self,cv_image):
        # aruco marker identification
        # if self.camera_matrix is None:
        #     return
        # getting the camera stream
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            self.get_logger().info(f"Marker id : {ids}")
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            aruco_distance = None
            marker_id = None
            for rvec, tvec, marker_id in zip(rvecs, tvecs, ids):
                cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
                self.get_logger().info(f"Marker {marker_id}: Position {tvec}, Rotation {rvec} x position: {tvec[0][2]}")
                aruco_distance = tvec[0][2]
                marker_id = marker_id[0]
                self.get_logger().info(f"marker id : {self.marker_id}")
            return aruco_distance,marker_id
        
        # cv2.imshow("Aruco Detection", cv_image)
    
    def line_segmentation(self,cv_image):
        # converting bgr to hsv
        hsv_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

        # blue color ranges in hsv
        lower_bound = np.array([114, 112, 17]) 
        upper_bound = np.array([159, 255, 118])

        # create a binary mask
        mask = cv2.inRange(hsv_img,lower_bound,upper_bound)

        # apply the mask to the original image
        blue_roi_img = cv2.bitwise_and(cv_image,cv_image,mask=mask)

        return blue_roi_img,mask
    
    def image_callback(self,msg):
        cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = cv2.resize(cv_image,(300,200))
        cv2.imshow("cv image ",img)
        res = self.aruco_detection(cv_image=cv_image)
        if res:
            self.aruco_distance,self.marker_id = res
        # getting right or left half the image based on aruco marker
        h,w = cv_image.shape[:2]
        # getting image right part to turn right
        if self.marker_id == 1 and self.aruco_distance < aruco_distance_threshold:
            self.get_logger().info(f"distance : {self.aruco_distance}")
            cv_image[:,:w//2] = 0
        # getting image left part to turn left
        elif self.marker_id == 0 and self.aruco_distance < aruco_distance_threshold:
            self.get_logger().info(f"distance : {self.aruco_distance}")
            cv_image[:,w//2:] = 0
        # stop the robot
        elif self.marker_id == 2 and self.aruco_distance < 2.0:
            cmd_msg = TwistStamped()
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.angular.z = 0.0
            self.pub_.publish(cmd_msg)
            return

        # line segmentation
        blue_roi_img,mask = self.line_segmentation(cv_image=cv_image)
        # getting line contour
        cmd_msg = TwistStamped()
        line = self.get_contours(mask)
        _,w = blue_roi_img.shape[:2]
        if line:
            x = line['x']
            error = x - w//2

            # PID controller calculation
            self.integral_error += error
            derivative_error = error - self.previous_error
            self.previous_error = error
            
            # calculating angular speed using PID 
            angular_speed = -(KP * error + KI * self.integral_error + KD * derivative_error)
            self.get_logger().info(f"error: {error}, angular speed: {angular_speed}")

            cv2.circle(blue_roi_img,(line['x'],line['y']),5,(0,0,255),7)
            # Publish velocity commands
            cmd_msg.twist.linear.x = LINEAR_SPEED
            cmd_msg.twist.angular.z = float(angular_speed)
            self.pub_.publish(cmd_msg)

        # publishing velocity commands to our robot
        # cmd_msg.twist.linear.x = LINEAR_SPEED
        # if error < 0:
        #     cmd_msg.twist.angular.z = float(error) * -KP
        # else:
        #     cmd_msg.twist.angular.z = float(error) * -KP
        # self.get_logger().info(f"angular speed : {cmd_msg.twist.angular.z} and error : {error}")
        # self.pub_.publish(cmd_msg)
        img = cv2.resize(blue_roi_img,(300,200))
        cv2.imshow("ROI image ",img)
        cv2.waitKey(1)

    def get_contours(self,mask):
        """Returns the centroid of the largest contour in the binary image (mask)"""
        MIN_AREA = 200

        # get list of contours 
        contours , _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        if contours:
            largest_contour = max(contours,key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if (M['m00'] > MIN_AREA):
                self.get_logger().info(f"M : {M}")
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])

                return {'x': x, 'y': y}
            
def main(args=None):
    rclpy.init(args = args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    