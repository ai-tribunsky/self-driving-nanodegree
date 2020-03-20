import time

import cv2
import numpy as np
from moviepy.editor import VideoFileClip
import matplotlib.pyplot as plt


class Detector(object):

    def __init__(self, camera, debug=False, debug_output_dir=''):
        self.camera = camera
        self.frame_w = camera.frame_w
        self.frame_h = camera.frame_h

        self.debug = debug
        self.debug_output_dir = debug_output_dir

    def cleanup(self):
        pass

    def process_video(self, src, dst, subclip=None):
        if subclip is None:
            clip = VideoFileClip(src)
        else:
            clip = VideoFileClip(src).subclip(subclip[0], subclip[1])

        white_clip = clip.fl_image(self.process_frame)
        white_clip.write_videofile(dst, audio=False)
        return dst

    def process_video_frame(self, src, t):
        frame = VideoFileClip(src).get_frame(t)
        return self.process_frame(frame)

    def process_frame(self, frame):
        times = {}
        start_time = time.time()

        # undistort frame
        undistorted_frame = self.camera.undistort2(frame, crop=False)
        if self.debug:
            times['undistort'] = time.time()

        # get lane lines pixels
        lane_pixels = self._get_lane_lines_pixels(undistorted_frame)
        if self.debug:
            times['lane_pixels'] = time.time()
            self._save_img(lane_pixels, 'lane_pixels')

        # build bird-eye-view
        bird_eye_view = self.camera.perspective_transform(lane_pixels)
        if self.debug:
            times['bird_eye_view'] = time.time()
            self._save_img(bird_eye_view, 'bird_eye_view')

        left_fit, left_x, left_y, right_fit, right_x, right_y = self._fit_lines(bird_eye_view)

        if self.debug:
            print('====')
            print('Process time')
            total = 0
            prev_time = start_time
            for k in times:
                time_k = times[k] - prev_time
                print(k, '=', time_k)
                total += time_k
                prev_time = time_k
            print('Total:', total)

        return bird_eye_view

    def _get_lane_lines_pixels(self, img):
        HLS = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        S = HLS[:, :, 2]
        V = HSV[:, :, 2]

        # gradient threshold
        S_gradient_binary = self._get_gradient_threshold_binary_img(S, (30, 255))
        if self.debug:
            self._save_img(S_gradient_binary, 'S_gradient_binary')

        V_gradient_binary = self._get_gradient_threshold_binary_img(V, (30, 255))
        if self.debug:
            self._save_img(V_gradient_binary, 'V_gradient_binary')
        gradient_filter = (S_gradient_binary == 1) | (V_gradient_binary == 1)

        # color filter
        V_color_binary = self._get_color_threshold_binary_img(V, (200, 255))
        if self.debug:
            self._save_img(V_color_binary, 'v_color_binary')

        S_color_binary = self._get_color_threshold_binary_img(S, (180, 255))
        if self.debug:
            self._save_img(S_color_binary, 's_color_binary')
        color_filter = (S_color_binary == 1) | (V_color_binary == 1)

        combined = np.zeros_like(S)
        combined[color_filter | color_filter] = 1

        return combined

    def _get_color_threshold_binary_img(self, img, threshold=(0, 255)):
        binary = np.zeros_like(img)
        binary[(img > threshold[0]) & (img <= threshold[1])] = 1
        return binary

    def _get_gradient_threshold_binary_img(self, img, threshold=(0, 255)):
        sobel_x = np.abs(cv2.Sobel(img, cv2.CV_64F, 1, 0))
        scaled_sobel = np.uint8(255 * sobel_x / np.max(sobel_x))

        binary = np.zeros_like(img)
        binary[(scaled_sobel > threshold[0]) & (scaled_sobel <= threshold[1])] = 1
        return binary

    def _fit_lines(self, img, windows=9, margin=100, minpix=50):
        left_x, left_w, right_x, right_w = self._get_lines_start_positions(img)
        if self.debug:
            print('Lane lines pos: x=%f, w=%f; x=%f, w=%f' % (left_x, left_w, right_x, right_w))
            out_img = np.dstack((img, img, img))

        window_height = self.frame_w // windows

        nonzero = img.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        left_x_current = left_x
        right_x_current = right_x

        left_lane_inds = []
        right_lane_inds = []

        for window in range(windows):
            win_y_low = self.frame_h - (window + 1) * window_height
            win_y_high = self.frame_h - window * window_height
            win_x_left_low = left_x_current - margin
            win_x_left_high = left_x_current + margin
            win_x_right_low = right_x_current - margin
            win_x_right_high = right_x_current + margin

            if self.debug:
                cv2.rectangle(
                    out_img,
                    (win_x_left_low, win_y_low),
                    (win_x_left_high, win_y_high),
                    (0, 255, 0),
                    2
                )
                cv2.rectangle(
                    out_img,
                    (win_x_right_low, win_y_low),
                    (win_x_right_high, win_y_high),
                    (0, 255, 0),
                    2
                )

            left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                         (nonzero_x >= win_x_left_low) & (nonzero_x < win_x_left_high)).nonzero()[0]
            right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                          (nonzero_x >= win_x_right_low) & (nonzero_x < win_x_right_high)).nonzero()[0]

            left_lane_inds.append(left_inds)
            right_lane_inds.append(right_inds)

            if len(left_inds) > minpix:
                left_x_current = np.int(np.mean(nonzero_x[left_inds]))
            if len(right_inds) > minpix:
                right_x_current = np.int(np.mean(nonzero_x[right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        left_x = nonzero_x[left_lane_inds]
        left_y = nonzero_y[left_lane_inds]
        left_fit = np.polyfit(left_y, left_x, 2)

        right_x = nonzero_x[right_lane_inds]
        right_y = nonzero_y[right_lane_inds]
        right_fit = np.polyfit(right_y, right_x, 2)

        if self.debug:
            plot_y = np.linspace(0, self.frame_h - 1, self.frame_h)
            left_fit_x = left_fit[0] * plot_y ** 2 + left_fit[1] * plot_y + left_fit[2]
            right_fit_x = right_fit[0] * plot_y ** 2 + right_fit[1] * plot_y + right_fit[2]
            out_img[left_y, left_x] = [255, 0, 0]
            out_img[right_y, right_x] = [0, 0, 255]
            plt.plot(left_fit_x, plot_y, color='yellow')
            plt.plot(right_fit_x, plot_y, color='yellow')
            plt.imshow(out_img)
            plt.title('Lane lines fit')
            plt.show()

        return left_fit, left_x, left_y, right_fit, right_x, right_y

    def _get_lines_start_positions(self, img):
        histogram = np.sum(img[self.frame_h // 2:, 200:self.frame_w - 100], axis=0)
        if self.debug:
            plt.plot(histogram)
            plt.title('Lane pixels hist')
            plt.show()

        midpoint = self.frame_w // 2
        left_x = np.argmax(histogram[:midpoint])
        left_w = histogram[left_x]
        right_x = midpoint + np.argmax(histogram[midpoint:])
        right_w = histogram[right_x]

        return left_x + 200, left_w, right_x + 200, right_w

    def _save_img(self, img, name):
        plt.imshow(img, cmap='gray')
        plt.title(name)
        plt.show()

        cv2.imwrite(self.debug_output_dir + '/' + name + '.jpg', img)
