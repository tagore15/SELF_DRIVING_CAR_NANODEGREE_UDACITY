#**Finding Lane Lines on the Road** 

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"
[pipe1]: ./write_images/gaussian.png "Gaussian"
[pipe2]: ./write_images/canny.png "Canny"
[pipe3]: ./write_images/region.png "Region"
[pipe4]: ./write_images/hough.png "Hough"

---

### Reflection

###1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps.  
1) Conversion of images to grayscale to get a single channel    

2) Applying gaussian blurring to remove noisy gradients and focus on strong contrast differences.  
We get below image after applying above 2 steps:-  
![gray image][pipe1]   

3) Do Canny edge detection to find boundary points as we get below output.     
![canny image][pipe2]   
  
4) mask region of interest to filter out boundaries other than lane lines.    
![region image][pipe3]   

5) finally we find left and lines through hough transformation on boundary points.   

In order to draw a single line on the left and right lanes, I modified the draw_lines() function by iterating through all hough lines and segregating them into left and right lanes. Then I calcuate average slope, center of left and right lines and plot left and right lines passing through respective centers within boundary.   
![region image][pipe4]   


###2. Identify potential shortcomings with your current pipeline


One potential shortcoming would be what would happen when lanes lines are not straight i.e. it has continuous curve.  

Another shortcoming could be detection of false lanes lines due to shadows. 


###3. Suggest possible improvements to your pipeline

A possible improvement would be to to use RGB normalization to mitigate effect of shadows.  

Another potential improvement could be to extrapolate a curve through centers of detected hough segements.
