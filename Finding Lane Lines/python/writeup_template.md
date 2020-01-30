# **Finding Lane Lines on the Road*- 

The goals / steps of this project are the following:
- Make a pipeline that finds lane lines on the road
- Reflect on your work in a written report


### 1. Pipeline
The implemented pipeline for a single frame processing consists of 5 steps:
 - image pre-processing
 - edges detection
 - edges post-processing
 - lines detection
 - lines filtering and clusterisation

#### 1.1 Image Pre-processing
 RGB image converts to grayscale image to simplify image analysis and shrink calculations dimensions.
 Then Gaussian blur and dilation are applied to eliminate noise and insignificant details detection in further step.
 ![Pre-processing: blur + dilation][test_images_output/dilated_solidYellowLeft.jpg]

#### 1.2 Edges detection
Canny edge detector with L2 norm is used to perform this step.
 ![Edges detection][test_images_output/edges_solidYellowLeft.jpg]

#### 1.3 Edges Post-processing
To improve Hought Transform algorithm result edges bluring is used
![Edges blur][test_images_output/blured_edges_solidYellowLeft.jpg]

#### 1.4 Lines Detection
The Probabilistic Hough Line Transform is used to detect straight lines.
Probabilistic version is more efficient then standard implementation.
Hought Transform applied not for all image - only for a region of interest
described with polygon with vertices: 
(1% of width, 100% of height), (10% of width, 50% of height), (90% of width, 50% of heigh) and (99% of width, 100% of height).
Region of interest is calculats empirically.

#### 1.5 Lines Filtering and Clusterisation
Hought Transform returns a lot of lines, but we need only to lines.
In this step horizontal lines, lines with a little slope are filters out.
Lines with similar slopes and intercepts are merged into clusters. 
Then average line is calculated for each cluster and extents to "region of interest" polygon borders.
![Lines][test_images_output/lines_solidYellowLeft.jpg]

### 2. Potential shortcomings and Improvments
Challenge video and other videos which are used for testing purposes (https://www.youtube.com/watch?v=7HaJArMDKgI, https://youtu.be/Of5pGj6Obzo?t=385) shows some drawbacks of current implementation and pipeline:
- Lanes with big curvature (closed, sharp turns (pins) as extreme version) can not be correctly detected and drawLines must be replaced by drawSplines. Closed turns are distinct issue, because we don't see big part of lines and maybe with should use some probalistic ways to build, draw guide lines. Also strongly curved lanes deacrease quality of lines filtering after Hough transform with fixed slope threshold.
- Lanes distortions: dense traffic (like in NY) when we don't see any lines, lane changing cars, hard sun, shadows are affects edge detector, sand, snow
- Lanes changes, overtakings by host car
- Lanes merging without lane marks
- Camera distortions, fixation problems can be a problem for a fixed region of interest

#### Test Examples
- Driving videos channel: https://www.youtube.com/channel/UCBcVQr-07MH-p9e2kRTdB3A/videos
- https://www.youtube.com/watch?v=7HaJArMDKgI - NY downtown driving (fixed camera)
 - https://youtu.be/7HaJArMDKgI?t=166 - weaker traffic, lanes changing
 - https://youtu.be/7HaJArMDKgI?t=184 - intersection
 - https://youtu.be/7HaJArMDKgI?t=223 - just hard situation
 - https://youtu.be/7HaJArMDKgI?t=315 - closed turn right
 - https://youtu.be/7HaJArMDKgI?t=335 - closed left turn
 - https://youtu.be/7HaJArMDKgI?t=348 - uphill
 - https://youtu.be/7HaJArMDKgI?t=372 - curvy road
 - https://youtu.be/7HaJArMDKgI?t=1340 - normal driving
- https://www.youtube.com/watch?v=25EgbhdVESE - Beverly Hills (fixed camera)
- https://www.youtube.com/watch?v=Cw0d-nqSNE8 - LA (fixed camera)
- https://www.youtube.com/watch?v=WHX2yx1FD-w - Singapore night driving (fixed camera)
- https://youtu.be/Of5pGj6Obzo?t=385 - Forest/mountains drivings (fixed camera)
 - https://youtu.be/Of5pGj6Obzo?t=1596 closed turns
 - https://youtu.be/Of5pGj6Obzo?t=3138 - long closed turns + hard intersection (exit) with left turn
- https://youtu.be/qWL3Vw7d7bM?t=125 - Hong Kong (fixed camera), dirty road, higly distortion env
- https://youtu.be/8SVyB8gNxbg?t=65 - Mississippi Coast, highway, clean roads, sunrise (fixed camera)

