#### import required modules
import csv
import cv2
import numpy as np
import sklearn
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
# modules for creating neural network
from keras.models import Sequential
from keras.layers import Lambda, Flatten, Dense
from keras.layers.convolutional import Conv2D, Cropping2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.core import Dropout
from keras import backend as K

# collect data
samples = []
CENTER_CORRECTION = 0.25
with open('../data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    header = True
    for line in reader:
        if header:
            header = False
            continue
        angle = float(line[3])

        # remove half of angles around zero to remove bias
        if abs(angle) < 0.1:
            if np.random.uniform() < 0.4:
                continue
        
        # center image
        samples.append((line[0], angle))

        # left image and apply steering correction
        samples.append((line[1], angle + CENTER_CORRECTION))

        # right image and apply steering correction
        samples.append((line[2], angle - CENTER_CORRECTION))

# split data in traning and testing set
train_samples, validation_samples = train_test_split(samples, test_size=0.2)

# function for augmenting the dataset
# flip the image
def flip(image, angle):
    image_flipped = np.fliplr(image)
    angle_flipped = -angle
    return (image_flipped, angle_flipped)

# change brightness of image
# taken hints from https://chatbotslife.com/using-augmentation-to-mimic-human-driving-496b569760a9
def changeBrigtness(image):
    image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image1 = np.array(image1, dtype = np.float64)
    
    random_bright = 0.4+np.random.uniform()
    random_bias   = np.random.randint(-15, 15)
    
    image1[:,:,2] = image1[:,:,2]*random_bright + random_bias
    image1[:,:,2][image1[:,:,2]>255]  = 255
    image1 = np.array(image1, dtype = np.uint8)
    image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    return image1

# data generator to avoid memory overfill and augment the dataset on the fly
def generator(samples, batch_size = 64):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, int(batch_size/2)):
            batch_samples = samples[offset:offset+int(batch_size/2)]

            images = []
            angles = []
            
            for (image, angle) in batch_samples:
                name = '../data/IMG/' + image.split('/')[-1]
                image_arr = cv2.imread(name)
                images.append(image_arr)
                angles.append(angle)
                
                # we are doubling the dataset here by taking modified image with actual image. 
                # flipping the images
                image_flip, new_angle = flip(image_arr, angle)

                # change brightness of image
                new_image = changeBrigtness(image_flip)

                images.append(new_image)
                angles.append(new_angle)                

            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)
train_generator = generator(train_samples, batch_size=64)
validation_generator = generator(validation_samples, batch_size=64)

def getModel():
    model = Sequential()
    # Preprocess incoming data, centered around zero with small standard deviation 
    model.add(Lambda(lambda x: x/127.5 - 1.,
                    input_shape=(160, 320, 3)))
                    #output_shape=(row, col, ch)))

    # trim image to only see section with road
    model.add(Cropping2D(cropping=((60, 40), (20, 20))))

    model.add(Conv2D(16, (5, 5), activation='relu', padding='same'))
    model.add(Conv2D(16, (5, 5), activation='relu', padding='same'))
    model.add(MaxPooling2D(pool_size=(2,2)))
    model.add(Conv2D(24, (5, 5), activation='relu', padding='same'))
    model.add(Conv2D(24, (5, 5), activation='relu', padding='same'))
    model.add(MaxPooling2D(pool_size=(2,2)))
    model.add(Conv2D(32, (5, 5), activation='relu', padding='same'))
    model.add(Conv2D(32, (5, 5), activation='relu', padding='same'))
    model.add(MaxPooling2D(pool_size=(2,2)))
    model.add(Conv2D(48, (3, 3), activation='relu', padding='same'))
    model.add(Conv2D(48, (3, 3), activation='relu', padding='same'))
    model.add(MaxPooling2D(pool_size=(2,2)))
    model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
    model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
    model.add(Flatten())
    model.add(Dropout(0.2))
    model.add(Dense(100, activation='relu'))
    model.add(Dropout(0.2))
    model.add(Dense(50, activation='relu'))
    model.add(Dropout(0.2))
    model.add(Dense(10, activation='relu'))
    model.add(Dense(1))
    return model

# train and test the model
with K.get_session():
    model = getModel()
    model.compile(loss='mse', optimizer='adam')
    hist = model.fit_generator(train_generator, steps_per_epoch= \
        (2*len(train_samples))/64, validation_data=validation_generator, \
        validation_steps=(2*len(validation_samples))/64, epochs=3, verbose=0)
    print(hist.history)
    model.save('model.h5')
