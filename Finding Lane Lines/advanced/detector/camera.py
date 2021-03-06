import glob

import cv2
import numpy as np


class Camera(object):
    """ Camera calibrator """

    distortion_coefficients = None
    camera_matrix = None
    camera_matrix_refined = None
    roi = None
    perspective_matrix = None
    perspective_matrix_inv = None
    perspective_dst_points = None

    def __init__(self, frame_w, frame_h):
        self.frame_w = frame_w
        self.frame_h = frame_h

        # calculate perspective matrices
        self.calculate_perspective_matrices(frame_w, frame_h)

    def calibrate(self, img_dir, grid):
        """ Calibrate camera """

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        r, c = grid
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....
        objp = np.zeros((r * c, 3), np.float32)
        objp[:, :2] = np.mgrid[0:r, 0:c].T.reshape(-1, 2)

        # arrays to store object points and image points from all the images
        obj_points = []  # 3d point in real world space
        img_points = []  # 2d points in image plane
        img_shape = None
        files = glob.iglob(img_dir + '/*.jpg')
        for name in files:
            gray = cv2.imread(name, cv2.IMREAD_GRAYSCALE)
            img_shape = gray.shape[::-1]

            # find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, grid, None)
            if ret:
                obj_points.append(objp)

                refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                img_points.append(refined_corners)

        ret, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, img_shape, None, None)
        mtx_refined, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, img_shape, 1, img_shape)

        self.distortion_coefficients = dist
        self.camera_matrix = mtx
        self.camera_matrix_refined = mtx_refined
        self.roi = roi

    def undistort(self, img, crop=True):
        """ Undistort image """

        if crop:
            dst = cv2.undistort(img, self.camera_matrix, self.distortion_coefficients, None, self.camera_matrix_refined)

            # crop the image
            x, y, w, h = self.roi
            return dst[y:y + h, x:x + w]

        return cv2.undistort(img, self.camera_matrix, self.distortion_coefficients, None, self.camera_matrix)

    def undistort2(self, img, crop=True):
        """
        Undistorts image with second approach
        This approach shows x2 speed up vs Camera.undistort
        """

        if crop:
            map_x, map_y = cv2.initUndistortRectifyMap(
                self.camera_matrix,
                self.distortion_coefficients,
                None,
                self.camera_matrix_refined,
                (self.frame_w, self.frame_h),
                cv2.CV_32FC1
            )
            dst = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

            # crop the image
            x, y, w, h = self.roi
            return dst[y:y + h, x:x + w]

        map_x, map_y = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.distortion_coefficients,
            None,
            self.camera_matrix,
            (self.frame_w, self.frame_h),
            cv2.CV_32FC1
        )
        return cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

    def calculate_perspective_matrices(self, w, h):
        # src_points = np.float32([
        #     [w / 2 - 55, h / 2 + 100],
        #     [w / 6 - 10, h],
        #     [w * 5 / 6 + 60, h],
        #     [w / 2 + 55, h / 2 + 100]
        # ])
        # dst_points = np.float32([
        #     [w / 4, 0],
        #     [w / 4, h],
        #     [w * 0.75, h],
        #     [w * 0.75, 0]
        # ])
        # src_points = np.float32([
        #     [581, 477],
        #     [699, 477],
        #     [896, 675],
        #     [384, 675]
        # ])
        # dst_points = np.float32([
        #     [384, 0],
        #     [896, 0],
        #     [896, 720],
        #     [384, 720]
        # ])

        src_points = np.float32([[0, 223], [1207, 223], [0, 0], [1280, 0]])
        dst_points = np.float32([[561, 223], [703, 223], [0, 0], [1280, 0]])

        self.perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        self.perspective_matrix_inv = cv2.getPerspectiveTransform(dst_points, src_points)
        self.perspective_dst_points = dst_points

    def perspective_transform(self, img):
        shape = img.shape
        return cv2.warpPerspective(img, self.perspective_matrix, (shape[1], shape[0]))

    def perspective_transform_back(self, img):
        shape = img.shape
        return cv2.warpPerspective(img, self.perspective_matrix_inv, (shape[1], shape[0]))
