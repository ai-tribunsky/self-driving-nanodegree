#!/usr/bin/python3

import argparse
import time
import os.path
import cv2
from detector.detector import Detector
from detector.camera import Camera


parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='Detector mode: image, test-images, video, camera-test', default='test-images')
parser.add_argument('-i', '--image', help='Image path for "image" mode')
parser.add_argument('-v', '--video', help='Video path for "video" mode')
parser.add_argument('-f', '--frame', help='Video frame time in seconds')
parser.add_argument('-wt', '--width', help='Image/frame width in px')
parser.add_argument('-ht', '--height', help='Image/frame height in px')
parser.add_argument('-d', '--debug', help='Debug mode', action="store_true", default=False)

args = parser.parse_args()
mode = args.mode
image = args.image
video = args.video
frame = args.frame
frame_w = int(args.width)
frame_h = int(args.height)
debug = args.debug

camera = Camera(frame_w=frame_w, frame_h=frame_h)
camera.calibrate('camera_cal', (9, 6))

if debug:
    print('==== Camera Calibration ===')
    print('Distortion coeff:')
    print(camera.distortion_coefficients)
    print('Camera Matrix:')
    print(camera.camera_matrix)
    print('Camera Matrix Refined:')
    print(camera.camera_matrix_refined)
    print('Perspective Transform:')
    print(camera.perspective_matrix)

    # test distortions
    print('== Distortions tests ==')
    img_files = [
        'test_images/straight_lines1.jpg',
        'test_images/test5.jpg'
    ]
    for img_file in img_files:
        print(img_file)
        basename = os.path.basename(img_file)
        img = cv2.imread(img_file)

        start_time = time.time()
        undistorted = camera.undistort(img)
        end_time = time.time() - start_time
        cv2.imwrite('output_images/und_croped_' + basename, undistorted)
        print('  Undistort Croped:', end_time)

        start_time = time.time()
        undistorted = camera.undistort(img, False)
        end_time = time.time() - start_time
        cv2.imwrite('output_images/und_' + basename, undistorted)
        print('  Undistort not croped:', end_time)

        start_time = time.time()
        undistorted = camera.undistort2(img)
        end_time = time.time() - start_time
        cv2.imwrite('output_images/und2_croped_' + basename, undistorted)
        print('  Undistort 2 Croped:', end_time)

        start_time = time.time()
        undistorted = camera.undistort2(img, False)
        end_time = time.time() - start_time
        cv2.imwrite('output_images/und2_' + basename, undistorted)
        print('  Undistort 2 not croped:', end_time)

