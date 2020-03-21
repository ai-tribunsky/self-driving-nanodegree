import time

import numpy as np
from detector.line import LaneLine


class Lane(object):
    left_lines = {}
    right_lines = {}

    left_line = None
    right_line = None

    time_diff = 1.0
    fit_thresholds = [0.001, 0.4, 150]
    curvature_min = 300
    curvature_max = 10000

    xm_per_px = 1
    ym_per_px = 1

    def __init__(self, buffer_size=30, debug=False):
        self.buffer_size = buffer_size
        self.debug = debug

    def update_lines(self, left_line, right_line, ego_pos):
        x_start_diff = right_line.x_start - left_line.x_start
        self.xm_per_px = 3.7 / x_start_diff
        self.ym_per_px = 30 / np.max(left_line.detected_y)

        # calculate curvatures and offsets
        left_line.curvature = self._get_line_curvature(left_line)
        right_line.curvature = self._get_line_curvature(right_line)
        left_line.offset = self._get_line_offset(left_line, ego_pos)
        right_line.offset = self._get_line_offset(right_line, ego_pos)

        # check lines
        left_line.is_detected = self.is_line_detected(left_line, self.left_lines)
        right_line.is_detected = self.is_line_detected(right_line, self.right_lines)

        # save detections
        current_time = time.time()
        if left_line.is_detected:
            self.left_line = left_line
            keys = list(self.left_lines.keys())
            keys_len = len(keys)
            if keys_len == self.buffer_size:
                keys.sort()
                del self.left_lines[keys[0]]
            self.left_lines[current_time] = left_line
        else:
            self.left_line = None

        if right_line.is_detected:
            self.right_line = right_line
            keys = list(self.right_lines.keys())
            keys_len = len(keys)
            if keys_len == self.buffer_size:
                keys.sort()
                del self.right_lines[keys[0]]
            self.right_lines[current_time] = right_line
        else:
            self.right_line = None

    def is_line_detected(self, line, previous_detections):
        fit = line.fit
        if fit is None:
            return False

        # check curvature
        if line.curvature < self.curvature_min or line.curvature > self.curvature_max:
            if self.debug:
                print('Left line invalid curvature:', line.curvature)
            return False

        # compare with previous detection
        current_time = time.time()
        last_line = None
        keys = list(previous_detections.keys())
        keys.sort()
        keys_len = len(keys)
        if keys_len > 0:
            last_detection_time = keys[keys_len - 1]
            if current_time - last_detection_time > self.time_diff:  # clear outdated detections
                previous_detections.clear()
            else:
                last_line = previous_detections[last_detection_time]
            if last_line is not None:
                fit_diff = np.abs(fit - last_line.fit)
                if np.any(fit_diff > self.fit_thresholds):
                    if self.debug:
                        print('Left line fit diff error:', fit_diff)
                    return False

                return True
        return True

    def get_best_left_line(self):
        return self._get_average_line(self.left_lines)

    def get_best_right_line(self):
        return self._get_average_line(self.right_lines)

    def _get_average_line(self, lines):
        fit = None
        x_fit = None
        y_fit = None
        curvatures = 0
        offset = 0
        for k in lines:
            line = lines[k]
            if fit is not None:
                fit = np.add(fit, line.fit)
            else:
                fit = line.fit

            if x_fit is not None:
                x_fit = np.add(x_fit, line.x_fit)
            else:
                x_fit = line.x_fit

            y_fit = line.y_fit
            curvatures += line.curvature
            offset += line.offset

        count = len(lines.keys())
        average = LaneLine()
        average.fit = np.divide(fit, count)
        average.x_fit = np.divide(x_fit, count)
        average.y_fit = y_fit
        average.curvature = curvatures / count
        average.offset = offset / count
        return average

    def _get_line_curvature(self, line):
        y = np.multiply(line.detected_y, self.ym_per_px)
        x = np.multiply(line.detected_x, self.xm_per_px)
        fit = np.polyfit(y, x, 2)
        y_eval = np.max(line.detected_y) * self.ym_per_px
        return np.power((1 + (2 * fit[0] * y_eval + fit[1]) ** 2), 1.5) / np.abs(2 * fit[0])

    def _get_line_offset(self, line, ego_pos):
        x, y = ego_pos
        return np.abs(x - line.x_fit[y]) * self.xm_per_px
