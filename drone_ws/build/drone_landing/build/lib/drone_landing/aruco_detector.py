import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs   = None

        # Compatible with both old and new OpenCV
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(
                                cv2.aruco.DICT_4X4_50)

        # OpenCV 4.7+ uses DetectorParameters()
        # Older OpenCV uses DetectorParameters_create()
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.use_new_api  = True
            self.detector     = cv2.aruco.ArucoDetector(
                                    self.aruco_dict, self.aruco_params)
            self.get_logger().info('Using new OpenCV ArUco API')
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_api  = False
            self.detector     = None
            self.get_logger().info('Using legacy OpenCV ArUco API')

        self.marker_size = 0.4   # metres — must match URDF visual
        self.target_id   = 0     # ArUco marker ID to track

        # Gazebo camera plugin publishes to /drone/downward_camera/...
        # (set by <camera_name>downward_camera</camera_name> in URDF)
        self.create_subscription(
            Image, '/drone/downward_camera/image_raw', self.image_cb, 10)
        self.create_subscription(
            CameraInfo, '/drone/downward_camera/camera_info',
            self.camera_info_cb, 10)

        self.pose_pub   = self.create_publisher(
                            PoseStamped, '/landing/target_pose', 10)
        self.detect_pub = self.create_publisher(
                            Bool, '/landing/marker_detected', 10)
        self.debug_pub  = self.create_publisher(
                            Image, '/landing/debug_image', 10)

        self.get_logger().info('ArUco detector ready — waiting for camera...')

    def camera_info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info('Camera calibration received')

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers — works with both old and new OpenCV
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)

        det = Bool()

        if ids is not None and self.target_id in ids.flatten():
            det.data = True
            self.detect_pub.publish(det)

            idx = list(ids.flatten()).index(self.target_id)

            # 3D object points for the marker corners
            obj_pts = np.array([
                [-self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0]
            ], dtype=np.float32)

            img_pts = corners[idx][0].astype(np.float32)

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)

            if ok:
                pose = PoseStamped()
                pose.header.stamp    = msg.header.stamp
                pose.header.frame_id = 'camera_link'
                pose.pose.position.x = float(tvec[0])
                pose.pose.position.y = float(tvec[1])
                pose.pose.position.z = float(tvec[2])

                # Rotation vector → quaternion
                R, _ = cv2.Rodrigues(rvec)
                tr = 1 + R[0, 0] + R[1, 1] + R[2, 2]
                pose.pose.orientation.w = float(np.sqrt(max(0, tr)) / 2)
                pose.pose.orientation.x = float(
                    np.copysign(
                        np.sqrt(max(0, 1 + R[0,0] - R[1,1] - R[2,2])) / 2,
                        R[2,1] - R[1,2]))
                pose.pose.orientation.y = float(
                    np.copysign(
                        np.sqrt(max(0, 1 - R[0,0] + R[1,1] - R[2,2])) / 2,
                        R[0,2] - R[2,0]))
                pose.pose.orientation.z = float(
                    np.copysign(
                        np.sqrt(max(0, 1 - R[0,0] - R[1,1] + R[2,2])) / 2,
                        R[1,0] - R[0,1]))

                self.pose_pub.publish(pose)

            # Draw detected markers + axes on debug image
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            if ok:
                cv2.drawFrameAxes(
                    frame, self.camera_matrix,
                    self.dist_coeffs, rvec, tvec, 0.2)
        else:
            det.data = False
            self.detect_pub.publish(det)

        # Always publish debug image
        dbg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
