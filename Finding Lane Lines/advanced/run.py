#!/usr/bin/python3

import argparse
import glob
import os.path
import time

import cv2
import matplotlib.pyplot as plt
from detector.camera import Camera
from detector.detector import Detector

# python3 run.py --width=1280 --height=720 --mode=image --image=test_images/test1.jpg --debug
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
detector = Detector(
    camera=camera,
    debug=debug,
    debug_output_dir=os.path.dirname(os.path.abspath(__file__)) + '/output_images'
)

if mode == 'image':
    frame = cv2.imread(image)
    result = detector.process_frame(frame)
    plt.imshow(result)
    plt.title(image)
    plt.show()
elif mode == 'images-test':
    for filename in glob.iglob('test_images/*.jpg'):
        frame = cv2.imread(filename)
        detector.cleanup()
        result = detector.process_frame(frame)
        plt.imshow(result)
        plt.title(filename)
        plt.show()
elif mode == 'video':
    video_name = os.path.basename(video)
    dst = 'output_videos/' + video_name
    if frame is None:
        detector.process_video(video, dst)
    else:
        frame = float(frame)
        result = detector.process_video_frame(video, frame)
        plt.imshow(result)
        plt.title('Frame: %s:%f' % (video_name, frame))
        plt.show()
elif mode == 'camera-test':
    print('==== Camera Calibration ===')
    print('Distortion coefficients:')
    print(camera.distortion_coefficients)
    print('Camera Matrix:')
    print(camera.camera_matrix)
    print('Camera Matrix Refined:')
    print(camera.camera_matrix_refined)
    print('Perspective Transform:')
    print(camera.perspective_matrix)

    img_files = [
        'test_images/straight_lines2.jpg',
        'test_images/test5.jpg'
    ]

    # test distortions
    print('== Distortions tests ==')
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

    # test perspective transformation
    print('== Perspective transform tests ==')
    for img_file in img_files:
        img = cv2.imread(img_file)
        perspective_matrix = camera.perspective_matrix
        warped = camera.perspective_transform(img)
        plt.subplot(121), plt.imshow(img), plt.title('Input')
        plt.subplot(122), plt.imshow(warped), plt.title('Output')
        plt.show()
