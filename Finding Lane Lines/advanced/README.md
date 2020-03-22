# Advanced Lane Finding Project

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Sanity check of detected lines.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

## Run detector
Detector runs as console application with entry point in [run.py](run.py), which parses CLI arguments, setups detector and run in specified mode.  

```
usage: run.py [-h] [-m MODE] [-i IMAGE] [-v VIDEO] [-f FRAME] [-wt WIDTH] [-ht HEIGHT] [-d]

optional arguments:
  -h, --help                  show this help message and exit
  -m MODE, --mode MODE        Detector mode: image, test-images, video, camera-test
  -i IMAGE, --image IMAGE     Image path for "image" mode
  -v VIDEO, --video VIDEO     Video path for "video" mode
  -f FRAME, --frame FRAME     Video frame time in seconds
  -wt WIDTH, --width WIDTH    Image/frame width in px
  -ht HEIGHT, --height HEIGHT Image/frame height in px
  -d, --debug                 Debug mode
```

Example for image detection: `python3 run.py --width=1280 --height=720 --mode=image --image=test_images/test1.jpg --debug`

Example for video detection: `python3 run.py --width=1280 --height=720 --mode=video --image=project_video`

## Result
### Test images
![Test 1](output_images/test1.jpg)
![Test 2](output_images/test2.jpg)
![Test 3](output_images/test3.jpg)
![Test 4](output_images/test4.jpg)
![Test 5](output_images/test5.jpg)
![Test 6](output_images/test6.jpg)
![Straight lines 1](output_images/straight_lines1.jpg)
![Straight lines 2](output_images/straight_lines2.jpg)

### Test video
[Project Video](output_videos/project_video.mp4)

[Challenge Video](output_videos/challenge_video.mp4)

[Harder Challenge Video](output_videos/challenge_video.mp4)

## Pipeline
### Step 0. Camera Calibration

Camera calibration is required step to get undistorted frames and to build correct perspective transform.

OpenCV function `calibrateCamera()` was used to get camera intrinsic matrix, distortion coefficients.
This function requires preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world (assuming the chessboard is fixed on the (x, y) plane at z=0),
and "image points, which represent corners points on a image side and they were found by OpenCV `findChessboardCorners`.

After getting camera matrix it was refined by OpenCV `getOptimalNewCameraMatrix`.

Camera calibration implemented in [Camera class](detector/camera.py)

### Step 1. Undistort input image 
There are two methods implemented for image undistrortion [undistort](detector/camera.py#L59) and [undistort2](detector/camera.py#L71) in [Camera class](detector/camera.py#L59).
 
undistort2 with rectification map technique shows better performance and used in this particular step.

Distorted image:
![Basic Image](test_images/test1.jpg)

Undistorted image:
![Undistorted Image](output_images/test1_undistorted.jpg)

### Step 2. Creating a thresholded binary image
The function is responsible for building thresholded binary image is [_get_lane_lines_pixels](detector/detector.py#L93) in [Detector](detector/detector.py) class

Extracting lane lines pixels was performed in following steps:
- converting to HSL color scheme
- calculating gradient magnitude and gradient detection on S and L channels
- thresholding by magnitude and direction
- merging results of thresholding on S and L channels

S channel gradient magnitude:

![S channel gradient magnitude](output_images/test1_s_gradient_mag.png)

S channel gradient direction:

![S channel gradient direction](output_images/test1_s_gradient_dir.png)

L channel gradient magnitude:

![L channel gradient magnitude](output_images/test1_l_gradient_mag.png)

L channel gradient direction:

![L channel gradient direction](output_images/test1_l_gradient_dir.png)

Combined:

![Combined binary image](output_images/test1_gradient_combined.png)

### Step 3. Birds-eye-view
To get birds-eye-view perspective transformation matrix calculation is needed. 

This matrix defined in [calculate_perspective_matrices](detector/camera.py#L102) function in [Camera](detector/camera.py) class.

Binary thresholded image after perspective transformation:

![Birds-eye-view](output_images/test1_bird_eye_view.png)

### Lane lines pixels detecting and fitting
Detection and fitting implemented in `_fit_lines` function in `advanced/detector/detector.py:161`
and uses histogram peaks to determine starting positions and sliding window algorithm to detect lines pixels.

Histogram:

![Histogram](output_images/test1_hist.png)

After lines pixels detection 2-nd polynomial fitting is performed:

![Polynomial fit](output_images/test1_lines_fit.png)

### Step 4. Determine curvatures and ego vehicle offsets

Radius of curvature calculates in meters around vehicle position by M.Bourne formula.

Implementation can be found in [_get_line_curvature](detector/lane.py#L135) function at [detector/lane.py:135](detector/lane.py#L135).

To convert calculation result from pixel space ot real-world space x-s and y-s must be set in meters.

To perform conversion from pixels to meters we assume that lane is about 30 meters long and 3.7 meters wide.

And conversation coefficients calculate as follows:
```python
x_start_diff = right_line.x_start - left_line.x_start  # difference in x between lan lines starting positions
xm_per_px = 3.7 / x_start_diff
ym_per_px = 30 / max_y                                 # max_y in particular case is height of visible area 
```

To calculate vehicle offsets we assume that vehicle position is in lower center of an image.

And we just calculate difference between ego position and lane lines starting positions and convert results with coefficients 
xm_per_px and ym_per_px described above.

### Step 5. Sanity check
Sanity check step is required to reject invalid detections which can be appear during hard illumination conditions,
road wrecks and extreme lane curvature.

Firstly lines with crashed polyfit coefficients are discarded.

Then line curvature compared against valid interval: 300-10000m. Invalid lines marked as not detected.

After that polyfit coefficients compared against previous detection if time difference with previous detection less than 1 second.
Coefficients difference should not exceed 0.001, 0.4, 150 respectively.
If time difference is more than 1s then all detection buffer is cleared.

Sanity checks are performed in [update_lines](detector/lane.py#L26) function at [detector/lane.py:26](detector/lane.py#L26)
 
### Step 6. Warping back and output visual information
Implemented in [_draw_lane](detector/detector.py#L267) function at [detector/detector.py:267](detector/detector.py#L267)

Lane boundaries drawing and visual information output takes 5 steps:
- getting average lane lines fitted x-s and y-s from detections buffer to get smooth lines
- drawing trapezoidal poly form according lines x-s and y-s on a bird-eye view image
- warping back by perspective transform with inverted perspective matrix `detector/camera.py:121`
- alpha-blending with source undistorted image
- text outputting with curvatures and offsets

## Discussion

There are a few issues with current implementation:
- slow gradient thresholding. It takes 200-250ms, which is very poor performance for real-time detection.
  Color thesholding and simpler gradients calculations may be used. 
- nearby lines like road wrecks, lanes separators like in challenge video are mislead sliding window algorithm
- hard sun still a big problem, shadows can be tackled with S and L channels combination
- sharp turns another big issues for sliding window algorithm. In most difficult case one of the lane lines disappears from camera view as in "harder" video 
  
