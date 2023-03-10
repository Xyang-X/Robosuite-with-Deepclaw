import cv2
import numpy as np
import pandas as pd
from dect import Detector
import yaml
import cv2.aruco as aruco
import math

default_config_dir = "/home/xy/pyproj/robosuite/dect/detector.yaml"


class RemoteDetector(object):

    def __init__(self, config_path=default_config_dir, marker_len=None, camera_id=None):
        self.config_path = config_path
        self._camera_id = self._readConfig('camera_id')
        if camera_id is not None: self._camera_id = camera_id
        self.camera = cv2.VideoCapture(self._camera_id)
        self._camera_width = self._readConfig("camera_width")
        self._camera_height = self._readConfig("camera_height")
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._camera_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._camera_height)
        self._marker_len = self._readConfig("marker_length")  # Length of marker in meters
        if marker_len is not None: self._marker_len = marker_len
        self._camera_matrix = np.array(self._readConfig("camera_matrix"))  # 3x3 camera intrinsic matrix
        self._camera_dist = np.array(self._readConfig("camera_dist"))  # vector of distortion coefficients

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
        self.parameters.adaptiveThreshConstant = 10
        self.last_pose = np.zeros(6)  # record the pose of marker from last frame
        self.last_ori = np.zeros([2, 3])
        self.pose_data_set = np.zeros([50, 6])
        self._avg_queue = np.zeros([15, 6])
        self._detc_confirm = False
        self.record = False
        self.quit = False
        self.reflec_mat = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        self.reflec_mat2 = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])

    def _readConfig(self, keyword):
        '''Read a specific parameter from config file'''
        # TODO
        with open(self.config_path, 'r') as config:
            data = yaml.safe_load(config)
            return data[keyword]

    def _action_modifier(self, action, t_coef, r_coef):
        '''filter the noise of action order and amplify the input action'''
        for i in range(3, 6):
            if 2 < abs(action[i]):
                action[3:6] = 0

        action[0:6] = self._avg_filter(action[0:6])

        if 0.015 > abs(action[2]):
            action[2] = 0
        for i in range(2):
            if 0.005 > abs(action[i]):
                action[i] = 0
        action[6] = self._grip_control(3, 5)
        action[0:3] *= t_coef
        action[3:6] *= r_coef
        return action

    def _avg_filter(self, pose_data):
        length = self._avg_queue.shape[0]
        avg_pose = np.zeros(6)
        if sum(self._avg_queue).all == 0:
            for i in range(length):
                self._avg_queue[i, :] = pose_data
        else:
            self._avg_queue[0] = pose_data
            self._avg_queue[1:length] = self._avg_queue[0:length - 1]
        for i in range(6):
            avg_pose[i] = sum(self._avg_queue[:, i]) / length
        return avg_pose

    def _grip_control(self, m1, m2):
        dist = np.sqrt(sum((self.pose_data[m1][0:3] - self.pose_data[m2][0:3]) ** 2))
        print(dist)
        if dist > 0.35:
            return -1
        else:
            return 1

    def _detect_marker(self):
        '''detect aruco markers and calculate poses '''
        kernel5 = np.array([-1, -1, -1, -1, -1, -1, 2, 2, 2, -1, -1,
                            2, 1, 2, -1, -1, 2, 2, 2, -1, -1, -1, -1,
                            -1, -1])
        kernel5 = kernel5.reshape(5, 5)
        ret, frame = self.camera.read()
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.filter2D(gray, -1, kernel5)

        # Detect the markers in the frame
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # Draw the detected markers on the frame
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        self.pose_data = dict()

        # Estimate the 6D pose of the marker if it is present
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05,
                                                                  self._camera_matrix, self._camera_dist)
            for index, rvec in enumerate(rvecs):
                cv2.drawFrameAxes(frame, self._camera_matrix, self._camera_dist, rvec, tvecs[index], self._marker_len)

            for i, tvec in enumerate(tvecs):
                self.pose_data[ids[i][0]] = np.append(tvec, rvecs[i])

        cv2.drawFrameAxes(frame, self._camera_matrix, self._camera_dist, self.last_pose[3:6], self.last_pose[0:3],
                          self._marker_len)
        # cv2.drawFrameAxes(frame, self._camera_matrix, self._camera_dist, np.zeros(3), np.zeros(3),
        #                   self._marker_len)
        frame = cv2.flip(frame, 1)
        cv2.imshow("Camera View", frame)

        key = cv2.waitKey(1)
        if key == ord('a'):
            self._detc_confirm = False
        elif key == ord('s'):
            self.record = not self.record
        elif key == ord('q'):
            self.quit = True

    def single_gripper_control(self):
        self._detect_marker()
        action = np.zeros(7)
        current_pose = np.zeros(6)
        det_num = 0
        z_jerk = False
        for key in self.pose_data:

            if key == 3:
                if self._ori_estimator(self.pose_data[key]):
                    self.pose_data[key][3:6] = self.last_ori[0]
                    z_jerk = True
                det_num += 1
            if key == 5:
                det_num += 1
        if det_num == 2:

            current_pose[0:3] = (self.pose_data[3][0:3] + self.pose_data[5][0:3]) / 2
            current_pose[3:6] = self.pose_data[3][3:6]
            if not z_jerk:
                self.last_ori[0] = self.pose_data[3][3:6]
                # self.last_ori[1] = self.pose_data[5][3:6]
            if self._detc_confirm:
                action[0:6] = self.last_pose - current_pose
                action = self._action_modifier(action, 200, 4)
            else:
                if not z_jerk:
                    self._detc_confirm = True
            self.last_pose = current_pose
        action = action.dot((np.append(self.reflec_mat, self.reflec_mat)))

        return action
