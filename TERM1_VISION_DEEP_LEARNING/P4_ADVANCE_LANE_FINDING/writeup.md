**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/distort.png "distored calibration image"
[image2]: ./output_images/cal1.png "undistorted camera calibration image"
[image3]: ./output_images/distort_lane.png "distorted road image"
[image4]: ./output_images/cal2.png "undistorted road image"
[image5]: ./output_images/binary.png "binary thresholded image"
[image6]: ./output_images/warp_original.png "warped original image"
[image7]: ./output_images/warp_perspective.png "warped transformed image"
[image8]: ./output_images/histogram.png "histogram"
[image9]: ./output_images/window.png "window for lane detection"
[image10]: ./output_images/poly.png "polygon fit in lane curvature"
[image11]: ./output_images/radius.PNG "radius equation"
[image12]: ./output_images/inverse.png "inverse perspective of detected lane back to orignal image"
[image13]: ./output_images/binary_warp.png "binary perspective transformation"
[video1]: ./project_output.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. 

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the camera calibration section of the IPython notebook "sol.ipynb".  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard corners detection through opencv function `findChessboardCorners`.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result:

ORIGINAL DISTORTED CALIBRATION IMAGE

![original distorted calibration image][image1]

UNDISTORTED CALIBRATION IMAGE

![undistorted calibration image][image2]


### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

Using opencv function `undistort` and camera calibration parameters found in last section, I undistorted the images in my pipeline. Consider below images:-

ORIGINAL DISTORTED ROAD IMAGE

![original distorted road image][image3]

UNDISTORTED ROAD IMAGE

![undistorted road image][image4]


#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding code is in `thresh_bin` function of `sol.ipynb`). Here's an example of my output for this step from undistorted road image in last step.

![binary thresholded image][image5]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform is in PERSPECTIVE TRANSFORM section of `sol.ipynb`. I have hardcoded the source and destination points as following:  

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 220, 700      | 300, 700      | 
| 1100, 700     | 1000, 700     |
| 680, 450      | 1000, 10      |
| 600, 450      | 300, 10       |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image as shown below.

ACTUAL IMAGAE    

![warped original image][image6]

WARPED IMAGE    

![warped transformed image][image7]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

For indentifying lane lines, I did following steps on below binary warped image after perspective transformation from last step. Take following sample binary image of bird eye view of lane   

![binary_image][image13]


1) First I take a histogram of bottom half of image and finds left and right points from center where pixel intensity is highest.

![histogram left and right lane][image8]

2) Then I start from bottom and slide windows from bottom to top for both left and right lanes. I have set number of windows to 9 and height of each window is 80 pixel. Now I iterate on each window and have margin of around 100 around center of windows. I keep finding points lying within windows and update center of next window as per mean of points in current window. Check below image for sliding windows.

![window][image9]

3) Finally I fit a polynomial through points found within windows. Check below output for an image fit with polynomial. 

![polygon fit][image10]


#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I found radius of curvature and position of vehicle with respect to center in `RADIUS OF CURVATURE` section of `sol.ipynb`. For finding radius I used following formula provided in lessons and also converted pixel number to meter in both x and y direction.   

![radius_curvature][image11]

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

Finally I applied inverse perspective transform on lane portion to merge it with original road image. Code is in INVERSE PERSPECTIVE TRANSFORM section of solution notebook `sol.ipynb`. Here is an example of my result on a test image:

![inverse_warp][image12]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

I combine all steps applied on a single image above in a pipeline applied on video frames. Here's a [link to my video result](./project_output.mp4).

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

In this project, we applied gradient and color thresholding to get binary image with possible edge positions. After getting birds eye view of lane for curvature detection, we fit a polynomial through detected lane points. We needed to check threshold range for gradients and also selected source and destination points for perspective transformation such that straight lines are parallel after transformation. We applied Red color threshold to detect white and yellow lines while removing shadows. We still need to take into account affect of larger shadows and glare to try on harder challenge videos where there could be break in lanes. We can smooth lane area in current frame using location of lane in previous frame as lane is continuous in subsequent frames. We can also finetune gradient and color thresholds to better detect lane portion of image while disregarding other sections.
