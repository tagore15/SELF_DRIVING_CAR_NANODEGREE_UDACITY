#Build a Traffic Sign Recognition Project   

** The goals / steps of this project are the following: **

* Load the data set 
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image_hist]: ./hist.png "Histogram"
[image1]: ./new_images/1.png "1st new image"
[image2]: ./new_images/2.png "2nd new image"
[image3]: ./new_images/3.png "3rd new image"
[image4]: ./new_images/4.png "4th new image"
[image5]: ./new_images/5.png "5th new image"

###Data Set Summary & Exploration  

####1. Basic summary of the data set.   

The size of training set is 34799    
The size of the validation set is 4410   
The size of test set is 12630   
The shape of a traffic sign image is 32 * 32 * 3, which represents image width and height as 32 and there are 3 color channels.   
The number of unique classes/labels in the data set is 43   

####2. Include an exploratory visualization of the dataset.   

Here is an exploratory visualization of the data set. It is a histogram depicting distribution for count of different categories. 

![histogram of count of different categories][image_hist]

###Design and Test a Model Architecture   

####1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc.    

I did following things to preprocess the data and make it amenable to feed in neural network

#####a. Augment the dataset by rotation    
I doubled the size of our dataset by rotating each image by few degrees. We would get better accuracy as size of our training data is increased.

#####b. Convert to grayscale    
I convert from RGB to Grayscale so that our input image has a single channel and our neural network is simplified. 

#####c. Normalize the image   
I normalize the image in range [0, 1] so that learning is faster.  

After preprcessing, our training data set has size of (69598, 32, 32, 1)

####2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

My final model consisted of the following layers:  

Input           32x32x1   

Convolution     5x5	1x1 stride, VALID padding, outputs 28x28x12    
RELU   
Max pooling     2x2 stride, outputs 14x14x12   

Convolution 	5x5	1x1 stride, VALID padding, outputs 10x10x24   
RELU   
Max pooling		2x2 stride, outputs 5x5x24    

Flatten			outputs 1X600   

FullyConnected 	Size: 240   
RELU   
DropOut   

FullyConnected 	Size: 120   
RELU   
DropOut   

FullyConnected 	Size: 84   
RELU    
DropOut   

Output 			Size: 43   

Above consists of 2 convolution layers followed by 3 fully connected layers. We are using Dropout after fully connected layers for regularization.

####3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

To train the model, I use following hyperparameters  

LEARNING RATE = 0.0025  
EPOCHS = 15  
BATCH SIZE = 128  
KEEP_PROB = 0.80  (Drop out probability = 0.80)  

I used AdamOptimizer for optimization. We converted output to one hot encoded vector. At the end of each epoch, we see that our training and validation accuracy increasing and converge towards last epoch. 

####4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:

training set accuracy of 99.5%
validation set accuracy of 95.6%
test set accuracy of 94.3%

Initially accuracy of our model was 87% on simple LeNet Architecture. As Lenet architure need to do less complex task of classifying digits, I updated LeNet architecture to get better accuracy on this dataset. I imporved it by using additional fully connected layer and increasing depth of convolutional layers. 
Initially our model was overfitting by getting high accuracy on training data, but poor accuracy on validation set. For this, I put drop outs between different fully connected layers. 
I tried with different learning rates and found best accuracy with rate of 0.0025. We also tried different size of convolution layers. 

After tuning our model, We get high validation and testing accuracy > 94% required for this project.

###Test a Model on New Images

####1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:

![first new image][image1]    
![second new image][image2]   
![third new image][image3]    
![fourth new image][image4]   
![fifth new image][image5]   

####2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:  
- head only   
- Wild animals crossing   
- Pedestrians   
- Road work   
- Dangerous curve to the left   

Above all are correct predictions. Thus our model was able to correctly guess all images which amount to perfect 100% accuracy.

####3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

We are getting very high probability close to 1.0 of predicted signs for all images. Softmax probability for top 5 images is listed below:-

For 1st image,  we can see all other probabilities are nearly zero.    
[1.00000000e+00, 4.97247021e-10, 2.75059212e-11, 1.61488756e-11, 2.36237700e-12]

For 2nd image, matching probability is 0.96    
[9.65157092e-01, 3.48428302e-02, 5.96068546e-08, 2.01016044e-11, 2.79509163e-12]

For 3rd image, probability of matching to actual sign is 0.99   
[9.99999523e-01, 4.73475268e-07, 8.66439631e-09, 1.55775745e-17, 8.22112063e-18]

For 4th image, probability of matching to actual sign is nearly perfect 1.0   
[1.00000000e+00, 1.52848063e-08, 9.76909220e-09, 3.06293169e-09, 9.76841719e-10]

For 5th image, probability of matching to actual sign is nearly perfect again high 0.99   
[9.99988198e-01, 1.18417156e-05, 1.29803590e-08, 1.40897016e-11, 4.82681820e-12]
