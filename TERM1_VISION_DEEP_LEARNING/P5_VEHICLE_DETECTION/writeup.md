**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./output_images/car_non_car.png
[image21]: ./output_images/hog0.png
[image22]: ./output_images/hog1.png
[image23]: ./output_images/hog2.png
[image3]: ./output_images/sliding_window.png
[image4]: ./output_images/sliding_limit.png
[image51]: ./output_images/pipeline1.png
[image52]: ./output_images/pipeline2.png
[image53]: ./output_images/pipeline3.png
[image54]: ./output_images/pipeline4.png
[image55]: ./output_images/pipeline5.png
[image56]: ./output_images/pipeline6.png
[video1]: ./project_output.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  

You're reading it!

###Histogram of Oriented Gradients (HOG)

####1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for this step is contained in the function `get_hog_features` defined in FEATURE EXTRACTION section of the `sol.ipynb`.

I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![sample car and non-car images][image1]

I then explored different color spaces and different `skimage.hog()` parameters (`orientations`, `pixels_per_cell`, and `cells_per_block`).  I extracted hog features on above images and plotted them to get a feel for what the `skimage.hog()` output looks like.

Here is an example using the `YCrCb` color space and HOG parameters of `orientations=8`, `pixels_per_cell=(8, 8)` and `cells_per_block=(2, 2)` for all channels of image:
![hog features][image21]

![hog features][image22]

![hog features][image23]

####2. Explain how you settled on your final choice of HOG parameters.

I tried various combinations of parameters and found below parameters give good accuracy with faster training time. 
I found that with lower count of pixels_per_cell, running time and memory resources for hog extraction are high. I also found that using 3 color channels gave better accuracy than single channel hog features.  

Finally I settled on following hog features:-
* orientations    = 9
* pixels_per_cell = (8, 8)
* cells_per_block = (2, 2)
* color space = "YCrCb"
* channel used = "All" 

####3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

Training classifer code can be found at `TRAINING CLASSIFIER` section of `sol.ipynb`. 
For classification, I used SVM (Support Vector Machine) classifier with default options. I used following features for classifier. 

1) Binned color  

Here I resize the image to (32, 32) and then use its flattened array

2) Color Histogram  

Create a histogram of intensity in each channel with 32 bins and concatenate all these channels.

3) HOG  

I create hog of each channel as discussed in hog section of this document.

I standardize the data by using `StandardScaler` module of `sklearn.preprocessing`.   
I randomly split training and testing data (20%) for cross-validation. After training SVM on train data, we find accuracy of more than 98% on testing data. 

###Sliding Window Search

####1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I slide windows of size 64 * 64 across the image. In each portion of sliding window, I extract features and check for cars using trained model. I collect all car predicting boxes. I maintain an overlap of 50% between different windows which was able to give car detections with good runtime performance.   
To find cars with varying view sizes, I resize the image with different scales 1.5, 1.8 and 2.0 and then run slide windows. When I scaled my image with ratio below 1, it was taking high time and accuracy was not good with multiple false detections so I only scale greater than 1.0.  
Implementation of sliding windows can be found at `find_cars` function of `sol.ipynb`.  
Below is sample image of boxes found through sliding windows.

![sliding windows][image3]

####2. Show some examples of test images to demonstrate how your pipeline is working. What did you do to optimize the performance of your classifier?

I create HOG of entire image once and just take its subsample on each sliding window to improve runtime performance. 
To overcome false detections, I limited search area of sliding windows in between road lanes as shown in below image. This also improved performance of my pipeline.   

![sliding limit][image4]

Also I create a heatmap as per count of boxes in each pixel and apply a threshold limit so that false detections are filtered out. Output of my pipeline for different test images is depicted below.  

![pipeline][image51]
![pipeline][image52]
![pipeline][image53]
![pipeline][image54]
![pipeline][image55]
![pipeline][image56]

---

### Video Implementation

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)    
Here's a [link to my video result](./project_output.mp4)


####2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video.  From the positive detections, I created a heatmap and then thresholded that map to identify vehicle positions.  I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap. I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  
Check pipeline output image in last section.

I also keep a dequeue for last 10 frames and take their sum with a threshold to create smooth bouding box and avoid jitters in video. It also avoid glitches in car detection.

---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

As shown in video output, Pipeline mentioned in this document is able to detect cars with high accuracy in most of frames.  
We still can try to mitigate below issues:-

1) To not detect cars coming from opposite directions.   
2) Time lag between when car is able to get detected after entry into frame.    
3) separate bounding box of overlapping cars

To make pipeline more robust, we can try following actions
1) Using Deep Learning
2) Better Thersholds, hyperparameters
3) We can augment training data of vehicles and non-vehicles
4) Try on different videos and road conditions
