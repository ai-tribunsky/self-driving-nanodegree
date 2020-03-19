import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-m', '--mode', help='Detector mode: image, test-images, video', default='test-images')
parser.add_argument('-i', '--image', help='Image path for "image" mode')
parser.add_argument('-v', '--video', help='Video path for "video" mode')
parser.add_argument('-f', '--frame', help='Video frame time in seconds')
parser.add_argument('-w', '--width', help='Image/frame width in px')
parser.add_argument('-h', '--height', help='Image/frame height in px')
parser.add_argument('-d', '--debug', help='Debug mode', action="store_true", default=False)

args = parser.parse_args()

