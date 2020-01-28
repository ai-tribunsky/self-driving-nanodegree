# **Finding Lane Lines on the Road*- 

The goals / steps of this project are the following:
- Make a pipeline that finds lane lines on the road
- Reflect on your work in a written report


### 1. Pipeline
- Read one frame from a stream
- Get one channel or grayscale image
- Try to dermine potential road segments: by hardcoding or by image segmentation
- Get edges of the scene: blur + canny, or probalistic detection
- Find line segments, lines points by Hough transformation
- Filter-out lines points not intersted for us: horizontal lines, lines not in our lane
- Clasterize lines, points
- Fit polynomial (quadratic) or line + polynomial segments
- Maybe additional connection of segments will be required for intersections, roads exits
 - Determine direction of moving by comparing images features and visual odometry
- Write out one frame into output stream


### 2. Potential shortcomings


- Closed, sharp turns (pins)
- Lanes curvature
- Lanes distortions: lane changing cars, sand, snow, wet reflections, hard sun, shadows
- Chippers as lanes delimitor
- Lanes changes, overtakings 
- Lanes merging
- Camera distortions, fixation problems
 -  On-head camera + evening + night driving: https://www.youtube.com/watch?v=wRrK3RREHZk
- Bad road, vibrations


### 3. Possible improvements


### Test examples
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

