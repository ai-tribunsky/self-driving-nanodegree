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
    offset = None
