**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Preprocess and augment the data to train the model 
* Build a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./images/center.png "Center camera Image"
[image2]: ./images/left.png "Left Camera Image"
[image3]: ./images/right.png "Right Camera Image"
[image4]: ./images/crop.png "Cropped image" 
[image5]: ./images/flip.png "flipped image" 
[image6]: ./images/bright.png "brightness image"

## Rubric Points
###Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.

---
###Files Submitted & Code Quality

####1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolutional neural network
* video.mp4 for video of simulator when car is able to complete track successfully
* writeup_report.md summarizing the results

####2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

####3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

###Model Architecture and Training Strategy

####1. An appropriate model architecture has been employed

My model consists of a convolution neural network with number of 5*5 and 3*3 size convolutional filters with varying number of output depths (16, 24, 32, 48 and 64). These are followed by several fully connected layers of output sizes 100, 50, 10 and 1.
The model includes RELU layers to introduce nonlinearity, and the data is normalized in the model using a Keras lambda layer. Keras cropping layer is used to reduce size of images and generator is used to feed the training and validation data to architecture avoiding memory exhaustion.
Please check definition of getModel function in model.py for code of full model architecture. It is also discussed in Model Architecture Documentation section of this document.

####2. Attempts to reduce overfitting in the model

The model contains dropout layers in order to reduce overfitting. 

The model was trained and validated on different data sets to ensure that the model was not overfitting. The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

####3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 137).

####4. Appropriate training data

I used the Udacity provided dataset to train the model. 

For details about how I preprocessed and augmented the training data, see the next section. 

###Model Architecture and Training Documentation

####1. Solution Design Approach

The overall strategy for deriving a model architecture was to start with simple network architecture and incrementally improve it alongwith augmenting and processing the training data. 

My first step was to use a convolution neural network model similar to the LeNet-5. I thought this model might be appropriate because it would be able to classify internally different kind of road conditions ahead and decide steering angles accordingly. 

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. I found that car was not able to progress much and loss was high. Then I normalized the data and used a cropping layer to remove non road sections of image. I also increased the size and layers of convolutional filter to take it closer to Nvidia model. 
After increasing depth of model, car was running smooth with low loss but it was not able to cross grassy curve after bridge.
Finally I improved my training data by first removing bias towards zero angle steering by removing more than half of images with low steering angles. Secondly I doubled the useful training data by flipping and random brightness change over the images. I also put the dropout unit between different fully connected layer to avoid overfitting. After these changes, car was able to successfully complete the track at speed of 20.

####2. Final Model Architecture

The final model architecture consisted of a convolution neural network with the following layers and layer sizes:

Input: 160 X 320 X 3  
Lambda (Normalization) Layer  
Cropping Layer: output size: 60 X 280 X 3  
convolution size:5 X 5 output size: 60 X 280 X 16 padding:"same"  activation = 'relu'  
convolution size:5 X 5 output size: 60 X 280 X 16 padding:"same" activation='relu'  
maxPooling size: 2 X 2  
convolution size:5 X 5 output size: 30 X 140 X 24 padding:"same" activation='relu'  
convolution size:5 X 5 output size: 30 X 140 X 24 padding:"same" activation='relu'  
maxPooling size: 2 X 2  
convolution size:5 X 5 output size: 15 X 70 X 32 padding:"same" activation='relu'  
convolution size:5 X 5 output size: 15 X 70 X 32 padding:"same" activation='relu'  
maxPooling size: 2 X 2  
convolution size:5 X 5 output size: 7 X 35 X 48 padding:"same" activation='relu'  
convolution size:5 X 5 output size: 7 X 35 X 48 padding:"same" activation='relu'  
maxPooling size: 2 X 2  
convolution size:5 X 5 output size: 3 X 17 X 64 padding:"same" activation='relu'  
convolution size:5 X 5 output size: 3 X 17 X 64 padding:"same" activation='relu'  
Flatten Output: 1 X 3264    
DropOut (prob: 0.2)   
Fully Connected Layer output size: 100 activation: 'relu'   
DropOut (prob: 0.2)   
Fully Connected Layer output size: 50 activation: 'relu'   
DropOut (prob: 0.2)   
Fully Connected Layer output size: 10 activation: 'relu'   
Fully Connected Layer output size: 1   


####3. Creation of the Training Set & Training Process

I used Udacity provided training dataset.  

Below are some sample images from this dataset.

CENTER CAMERA IMAGE   
![center camera image][image1]
  

LEFT CAMERA IMAGE   
![left camera image][image2]

  
RIGHT CAMERA IMAGE    
![right camera image][image3]

To use left and right camera images, I apply steering adjustment of 0.25 around center. 

To get better driving output, I cropped the image for road only section reducing dimensions as shown below  

![cropped image][image4]

I also flipped images and angles to introduce postive angle images. For example, here is an image that has then been flipped:  

![flipped image][image5]

I also introduced random brightness in images. Here is one of such image.  

![bright image][image6]

I had around 15K images after filtering out more than half of images with low steering angles and including left and right camera images. After augmenting with flipping and random brightness my number of training data points is doubled.   

I finally randomly shuffled the data set and put 20% of the data into a validation set.   

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The number of epochs was chosen as 3 after various experiments to find converging value of loss on validation data. I used an adam optimizer so that manually training the learning rate wasn't necessary.
