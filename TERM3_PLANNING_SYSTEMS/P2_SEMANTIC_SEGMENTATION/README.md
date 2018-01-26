# Semantic Segmentation
### Introduction
In this project, you'll label the pixels of a road in images using a Fully Convolutional Network (FCN).

### Setup
##### Frameworks and Packages
Make sure you have the following is installed:
 - [Python 3](https://www.python.org/)
 - [TensorFlow](https://www.tensorflow.org/)
 - [NumPy](http://www.numpy.org/)
 - [SciPy](https://www.scipy.org/)
##### Dataset
Download the [Kitti Road dataset](http://www.cvlibs.net/datasets/kitti/eval_road.php) from [here](http://www.cvlibs.net/download.php?file=data_road.zip).  Extract the dataset in the `data` folder.  This will create the folder `data_road` with all the training a test images.

### Start
##### Implement
Implement the code in the `main.py` module indicated by the "TODO" comments.
The comments indicated with "OPTIONAL" tag are not required to complete.
##### Run
Run the following command to run the project:
```
python main.py
```

### Submission
1. Ensure you've passed all the unit tests.
2. Ensure you pass all points on [the rubric](https://review.udacity.com/#!/rubrics/989/view).
3. Submit the following in a zip file.
 - `helper.py`
 - `main.py`
 - `project_tests.py`
 - Newest inference images from `runs` folder

# Project Write Up

## Overview

This project focuses on using a fully convolutional neural network to classify the pixels of a given image as drivable road or non-drivable road. 

### Pre-Trainined VGG Model

To expedite the training of the neural network, the pre-trained weights from the standard VGG-16 model were used as the encoder of the fully convolutional network. The intention of the encoder is to classify the pixels of an image while maintaining spatial information. The VGG-16 model is primarily used to classify images from ImageNet-like dataset, therefore it can be used to classify pixels for sematic segmantation. Using the pre-trained version of the VGG-16 network reduces the need to train a fully convolutional network (FCN) as it already excels at image classification.

### FCN Architecture

The FCN architecture utilizes the basic VGG-16 structure as the encoder of the FCN. The decoder of the FCN consists of using transpose layers and skip layers in the neural network architecture. The transpose layers are used to decode the classification information and the skip layers are intended to maintain spatial information that may be lost during the encoding porition of the network. For this project, layers 3, 4, and 7 were extracted from the VGG network and used to create the skip layers and the transpose layers for the FCN.

Decoder:
```
layer3 = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, padding='SAME',
                          kernel_initializer=tf.truncated_normal_initializer(stddev = 1e-3))

layer4 = tf.layers.conv2d(vgg_layer4_out, num_classes, 1, padding='SAME',
                          kernel_initializer=tf.truncated_normal_initializer(stddev = 1e-3))

layer7 = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, padding='SAME',
                          kernel_initializer=tf.truncated_normal_initializer(stddev = 1e-3))

layer7_transpose = tf.layers.conv2d_transpose(layer7, num_classes, 4, strides=(2,2), padding='SAME',
                                              kernel_initializer=tf.truncated_normal_initializer(stddev = 1e-3))

skip_layer4 = tf.add(layer4, layer7_transpose)

layer4_transpose = tf.layers.conv2d_transpose(skip_layer4, num_classes, 4, strides=(2,2), padding='SAME',
                                              kernel_initializer=tf.truncated_normal_initializer(stddev = 1e-3))

skip_layer3 = tf.add(layer3, layer4_transpose)

layer3_transpose = tf.layers.conv2d_transpose(skip_layer3, num_classes, 16, strides=(8,8), padding='SAME',
                                              kernel_initializer=tf.truncated_normal_initializer(stddev = 1e-3))
```
### Training Process

Once the network architecture was generated, the training process for the network was initiated with a dataset of road images. Each pixel of the road images were labeled True for drivable road and False for non-drivable road. The dataset comes from the Kitti Road Data.

The labeled images were next feed into a training process that utilized a cross entropy loss function with a learning rate of 1e-4. This value was chosen from previous models used for behavioral cloning (Term 1, Project 3) and from a discussion with fellow students. The images that were processed and the FCN model were large, so a batch size of 3 was used to allow the GPU on my machine to handle the load. Finally, 7 EPOCHs were used to train the model.
```
def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    with sess.as_default():

        sess.run(tf.global_variables_initializer())

        print("Training model...")
        print()

        for i in range(epochs):

            for image, label in get_batches_fn(batch_size):

                _, loss = sess.run([train_op, cross_entropy_loss],
                feed_dict = {input_image: image, correct_label: label, keep_prob: .7, learning_rate: 1e-4})

            print("EPOCH {} ...".format(i+1))
            print("Loss = {:.4f}".format(loss))
            print()
```
## Results

During testing, the FCN architecture performed reasonably well. Most images are labeled correctly as seen in the example below:

![good_img](https://github.com/mblomquist/Udacity-SDCND-Term3/blob/master/P2/runs/1504588362.3873236/um_000032.png)

However, there were a few images that generated erratic boundaries as seen in the following example:

![bad_img](https://github.com/mblomquist/Udacity-SDCND-Term3/blob/master/P2/runs/1504588362.3873236/um_000078.png)

However, most images were classified with a good fidelity and would be acceptable for use as an additional layer of confidence in the acceptable portion of drivable road.

## Additional Work

Additional work can be put into this project to improve the following areas:

1. Network Processing Speed
2. Accuracy of Classification
3. Identification of Multiple Classes

The network processing speed would need to be increased for deployment in an actual self-driving car. With my current hardware, the network can process a single image in around 7 seconds. This would end up being quite slow in a real-world environment. Therefore, improving the speed of the network would become a very important aspect for deployment.

The network accuracy for classifying road/non-road can also be improved to provide a more-accurate representation of the world. This can be done by augementing the data provided during training of the neural network or adjusting the model architecture. 

Adding additiona classes for the network to identify would yield a more-accurate representation of the real-world. Other vehicles, pedestrians, and even street signs could be identified by the network to provide information to the path planner of the self-driving car. This information could then be used in the decision making process and combined with other data-types like LIDAR or RADAR data to generate a high-confidence representation of the real-world.
