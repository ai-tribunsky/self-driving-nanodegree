import numpy as np


class LaneLine(object):
    is_detected = False
    x_start = None

    fit = []
    x_fit = []
    y_fit = []

    detected_x = []
    detected_y = []

    curvature = None

    def get_curvature(self, xm_per_px, ym_per_px):
        if self.curvature is None:
            y = np.multiply(self.detected_y, ym_per_px)
            x = np.multiply(self.detected_x, xm_per_px)
            fit = np.polyfit(y, x, 2)
            y_eval = np.max(self.detected_y) * ym_per_px
            self.curvature = np.power((1 + (2 * fit[0] * y_eval + fit[1]) ** 2), 1.5) / np.abs(2 * fit[0])

        return self.curvature

