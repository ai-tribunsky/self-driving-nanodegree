import time

import cv2
import numpy as np
from moviepy.editor import VideoFileClip
import matplotlib.pyplot as plt
from detector.lane import Lane
from detector.line import LaneLine


class Detector(object):

    def __init__(self, camera, debug=False, debug_output_dir=''):
        self.camera = camera
        self.frame_w = camera.frame_w
        self.frame_h = camera.frame_h

        self.roi_w = self.frame_w
        self.roi_h = 223
        self.lane = Lane(self.roi_w, self.roi_h, buffer_size=20, debug=debug)

        self.debug = debug
        self.debug_output_dir = debug_output_dir

    def cleanup(self):
        self.lane = Lane(self.frame_w, self.frame_h, buffer_size=20, debug=self.debug)

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

        # undistort frame
        start_time = time.time()
        undistorted_frame = self.camera.undistort2(frame, crop=False)
        times['undistort'] = time.time() - start_time
        if self.debug:
            self._show_img(undistorted_frame, 'undistorted_frame', cmap=None)

        # get roi
        hood_pos_y = self.frame_h - 47  # 47 is a hood height
        roi_start_y = hood_pos_y - self.roi_h
        roi = undistorted_frame[roi_start_y:hood_pos_y, :, :]
        if self.debug:
            self._show_img(roi, 'roi', cmap=None)

        # get lane lines pixels
        start_time = time.time()
        lane_pixels = self._get_lane_lines_pixels(roi)
        times['lane_pixels'] = time.time() - start_time
        if self.debug:
            self._show_img(lane_pixels, 'lane_pixels', 'gray')

        # build bird-eye-view
        start_time = time.time()
        bird_eye_view = self.camera.perspective_transform(lane_pixels)
        times['bird_eye_view'] = time.time() - start_time
        if self.debug:
            self._show_img(bird_eye_view, 'bird_eye_view', cmap='gray')

        # fit lines
        start_time = time.time()
        left_start_x, left_w, right_start_x, right_w = self._get_lines_start_positions(bird_eye_view[bird_eye_view.shape[0]//2:])
        left_line, right_line = self._fit_lines(bird_eye_view, left_start_x, right_start_x)
        times['fit_lines'] = time.time() - start_time
        if self.debug:
            print('Lane lines:')
            print('  x=%f, w=%f' % (left_start_x, left_w))
            print('  x=%f, w=%f' % (right_start_x, right_w))

        # calculate result lane lines properties
        start_time = time.time()
        self.lane.update_lines(left_line, right_line)
        times['update_lines'] = time.time() - start_time

        # draw lane boundaries and lane info
        start_time = time.time()
        frame_with_lane = self._draw_lane(bird_eye_view, undistorted_frame)
        times['drawing'] = time.time() - start_time

        print('\n\nProcess time:')
        total = 0
        for k in times:
            print(k, '=', times[k])
            total += times[k]
        print('Total:', total)

        return frame_with_lane

    def _get_lane_lines_pixels(self, img):
        HLS = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        L = HLS[:, :, 1]
        S = HLS[:, :, 2]
        if self.debug:
            self._show_img(L, 'L_channel_equal', 'gray')
            self._show_img(S, 'S_channel_equal', 'gray')

        # gradient threshold
        kernel = 11
        S_gradient_x = np.abs(cv2.Sobel(S, cv2.CV_64F, 1, 0, ksize=kernel))
        S_gradient_y = np.abs(cv2.Sobel(S, cv2.CV_64F, 0, 1, ksize=kernel))
        L_gradient_x = np.abs(cv2.Sobel(L, cv2.CV_64F, 1, 0, ksize=kernel))
        L_gradient_y = np.abs(cv2.Sobel(L, cv2.CV_64F, 0, 1, ksize=kernel))
        S_gradient_binary = self._get_gradient_magnitude_binary_img(
            S, S_gradient_x, S_gradient_y, threshold=(30, 255)
        )
        L_gradient_binary = self._get_gradient_magnitude_binary_img(
            L, L_gradient_x, L_gradient_y, threshold=(50, 255)
        )
        S_gradient_dir_binary = self._get_gradient_direction_binary_img(
            S, S_gradient_x, S_gradient_y, threshold=(0.7, 1.3)
        )
        L_gradient_dir_binary = self._get_gradient_direction_binary_img(
            L, L_gradient_x, L_gradient_y, threshold=(0.7, 1.3)
        )
        gradient_filter = ((S_gradient_binary == 1) & (S_gradient_dir_binary == 1)) | ((L_gradient_binary == 1) & (L_gradient_dir_binary == 1))
        if self.debug:
            self._show_img(S_gradient_binary, 'S_gradient_binary', 'gray')
            self._show_img(L_gradient_binary, 'L_gradient_binary', 'gray')
            self._show_img(S_gradient_dir_binary, 'S_gradient_dir_binary', 'gray')
            self._show_img(L_gradient_dir_binary, 'L_gradient_dir_binary', 'gray')
            self._show_img(gradient_filter, 'gradient_filter', 'gray')

        # color filter
        # L_color_binary = self._get_color_binary_img(L, (200, 255))
        # if self.debug:
        #     self._save_img(L_color_binary, 'L_color_binary')
        # S_color_binary = self._get_color_binary_img(S, (180, 255))
        # if self.debug:
        #     self._save_img(S_color_binary, 'S_color_binary')
        # color_filter = (S_color_binary == 1) | (L_color_binary == 1)
        # if self.debug:
        #     self._save_img(color_filter, 'color_filter')

        combined = np.zeros_like(S)
        combined[gradient_filter] = 1

        return combined

    def _get_color_binary_img(self, img, threshold=(0, 255)):
        binary = np.zeros_like(img)
        binary[(img > threshold[0]) & (img <= threshold[1])] = 1
        return binary

    def _get_gradient_magnitude_binary_img(self, img, gradient_x, gradient_y, threshold=(0, 255)):
        mag = np.sqrt(gradient_x ** 2 + gradient_y ** 2)
        mag = np.uint8(255 * mag / np.max(mag))

        binary = np.zeros_like(img)
        binary[(mag > threshold[0]) & (mag <= threshold[1])] = 1
        return binary

    def _get_gradient_direction_binary_img(self, img, gradient_x, gradient_y, threshold=(0, np.pi / 2)):
        direction = np.arctan2(gradient_y, gradient_x)

        binary_output = np.zeros_like(img)
        binary_output[(direction >= threshold[0]) & (direction <= threshold[1])] = 1
        return binary_output

    def _fit_lines(self, img, left_start_x, right_start_x, windows=6, margin=15, minpix=20):
        if self.debug:
            out_img = np.dstack((img, img, img))

        nonzero = img.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        left_x_current = left_start_x
        right_x_current = right_start_x

        left_lane_inds = []
        right_lane_inds = []

        h, w = img.shape
        window_height = h // windows
        for window in range(windows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
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

        y_fit = np.linspace(0, h - 1, h, dtype=np.float32)

        left_line = LaneLine()
        left_line.y_fit = y_fit
        left_line.x_start = left_start_x
        left_lane_inds = np.concatenate(left_lane_inds)
        if len(left_lane_inds) > 0:
            left_x = nonzero_x[left_lane_inds]
            left_y = nonzero_y[left_lane_inds]
            left_fit = np.polyfit(left_y, left_x, 2)
            left_line.fit = left_fit
            left_line.x_fit = left_fit[0] * y_fit ** 2 + left_fit[1] * y_fit + left_fit[2]
            if self.debug:
                out_img[left_y, left_x] = [255, 0, 0]
                plt.plot(left_line.x_fit, left_line.y_fit, color='yellow')

        right_line = LaneLine()
        right_line.y_fit = y_fit
        right_line.x_start = right_start_x
        right_lane_inds = np.concatenate(right_lane_inds)
        if len(right_lane_inds) > 0:
            right_x = nonzero_x[right_lane_inds]
            right_y = nonzero_y[right_lane_inds]
            right_fit = np.polyfit(right_y, right_x, 2)
            right_line.fit = right_fit
            right_line.x_fit = right_fit[0] * y_fit ** 2 + right_fit[1] * y_fit + right_fit[2]
            if self.debug:
                out_img[right_y, right_x] = [0, 0, 255]
                plt.plot(right_line.x_fit, right_line.y_fit, color='yellow')

        if self.debug:
            plt.imshow(out_img)
            plt.title('Lane lines fit')
            plt.show()

        return left_line, right_line

    def _get_lines_start_positions(self, img):
        histogram = np.sum(img, axis=0)
        if self.debug:
            plt.plot(histogram)
            plt.title('Lane pixels hist')
            plt.show()

        margin = 100
        midpoint = img.shape[1] // 2
        left_x = (midpoint - margin) + np.argmax(histogram[(midpoint - margin):midpoint])
        left_w = histogram[left_x]
        right_x = midpoint + np.argmax(histogram[midpoint:(midpoint + margin)])
        right_w = histogram[right_x]

        return left_x, left_w, right_x, right_w

    def _draw_lane(self, warped, dst_img):
        # calculate average lines
        left_line = self.lane.get_best_left_line()
        right_line = self.lane.get_best_right_line()

        # draw lane boundaries
        color_warp = np.zeros_like(warped).astype(np.uint8)
        color_warp = np.dstack([color_warp, color_warp, color_warp])

        pts_left = np.array([np.transpose(np.vstack([left_line.x_fit, left_line.y_fit]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_line.x_fit, right_line.y_fit])))])
        pts = np.hstack((pts_left, pts_right))
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
        roi_warped_back = self.camera.perspective_transform_back(color_warp)
        full_size_img = np.zeros_like(dst_img).astype(np.uint8)
        full_size_img[450:(450 + self.roi_h), :, :] = roi_warped_back
        img_with_lane = cv2.addWeighted(dst_img, 1, full_size_img, 0.3, 0)

        # display lines info
        cv2.putText(
            img_with_lane,
            'Curvature Radius: {}m'.format(np.round((left_line.curvature + right_line.curvature) / 2, 2)),
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )

        y = self.roi_h - 1
        left_x = left_line.x_fit[y]
        right_x = right_line.x_fit[y]
        lane_center = left_x + (right_x - left_x) / 2
        offset = (self.roi_w / 2 - lane_center) * self.lane.xm_per_px
        cv2.putText(
            img_with_lane,
            'Offset: {}m'.format(np.round(offset, 2)),
            (10, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )

        return img_with_lane

    def _show_img(self, img, name, cmap=None):
        if cmap is None:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.imshow(img, cmap=cmap)
        plt.title(name)
        plt.show()
