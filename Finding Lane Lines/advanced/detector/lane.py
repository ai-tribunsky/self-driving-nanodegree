import time

import numpy as np


class Lane(object):
    left_lines = {}
    right_lines = {}

    left_line = None
    right_line = None

    time_diff = 2.0
    fit_thresholds = [0.001, 0.4, 150]
    curvature_min = 400
    curvature_max = 10000

    def __init__(self, buffer_size=30, debug=False):
        self.buffer_size = 30
        self.debug = debug

    def get_ego_offsets(self, ego_pos, units='px'):
        x, y = ego_pos

        left_offset = x - self.left_line.start_x
        right_offset = x - self.right_line.start_x

        if units == 'm':
            x_start_diff = np.abs(left_offset - right_offset)
            xm_per_px = 3.7 / x_start_diff
            left_offset *= xm_per_px
            right_offset *= xm_per_px

        return left_offset, right_offset

    def update_lines(self, left_line, right_line):
        x_start_diff = right_line.start_x - left_line.start_x
        xm_per_px = 3.7 / x_start_diff
        ym_per_px = 30 / np.max(left_line.detected_y, right_line.detected_y)

        # check lines
        left_line.is_detected = self.is_line_detected(left_line, self.left_lines, xm_per_px, ym_per_px)
        right_line.is_detected = self.is_line_detected(right_line, self.right_lines, xm_per_px, ym_per_px)

        # save detections
        if left_line.is_detected:
            self.left_line = left_line
            self.left_lines
        else:
            self.left_line = None

        if right_line.is_detected:
            self.right_line = right_line
        else:
            self.right_line = None

    def is_line_detected(self, line, previous_detections, xm_per_px, ym_per_px):
        fit = line.fit
        if fit is None:
            return False

        current_time = time.time()

        # compare with previous detection
        last_line = None
        keys = previous_detections.keys()
        keys_len = len(keys)
        if keys_len > 0:
            last_detection_time = keys[keys_len - 1]
            if current_time - last_detection_time > self.time_diff:  # clear outdated detections
                previous_detections.clear()
            else:
                last_line = previous_detections[last_detection_time]
            if last_line is not None:
                fit_diff = np.abs(fit - last_line)
                if np.any(fit_diff > self.fit_thresholds):
                    if self.debug:
                        print('Left line fit diff error:', fit_diff)
                    return False

                return True

        # check curvature
        curvature = line.get_curvature(xm_per_px, ym_per_px)
        if curvature < self.curvature_min or curvature > self.curvature_max:
            if self.debug:
                print('Left line invalid curvature:', curvature)
            return False

        return True
