"""
Simple script that analyzes model accuracy at traffic lights prediction for various levels of cropping.
Crops are centered on image center
"""

import os
import glob
import pprint

import keras
import cv2
import tqdm
import numpy as np

import utilities


def process_single_image(image):
    """
    Process image to format suitable for model input
    :param image: numpy array
    :return: numpy array
    """

    desired_shape = (128, 128)
    resized_image = cv2.resize(image, desired_shape, cv2.INTER_LINEAR)
    scaled_image = resized_image.astype('float32') / 255

    return scaled_image


def crop_image_with_relative_margin(image, relative_margin):
    """
    Crops margin number of pixels on each side. Margin is computed based on relative size to image size in each direction
    :param image: numpy arrry
    :param relative_margin: float
    :return: numpy array
    """

    y_size = image.shape[0]
    y_margin = int(relative_margin * y_size)

    x_size = image.shape[1]
    x_margin = int(relative_margin * x_size)

    return image[y_margin:y_size - y_margin, x_margin:x_size - x_margin]


def get_processed_prediction(probabilities, red_confidence):
    """
    Given probabilities return predicted class if. Red light is predicted only if its probability is
    above red_confidence. Else all red predictions are classified as 'others'
    :param probabilities: numpy array
    :param red_confidence: threshold for accepting red predictions
    :return: class id, integer
    """

    raw_prediction = np.argmax(probabilities)

    # If prediction isn't red light, accept it
    if raw_prediction != 0:

        return raw_prediction

    else:

        # If prediction is for red light, accept it only if it's confident enough, else return 'others'
        return 0 if probabilities[0] > red_confidence else 3


def get_confusion_matrix(model, images_map, red_confidence):

    matrix = np.zeros(shape=(len(images_map.keys()), len(images_map.keys())))

    for true_class_id, images in images_map.items():

        # Some datasets, e.g. yellow lights, might be empty
        if len(images) > 0:

            processed_images = np.array([process_single_image(image) for image in images])
            batch_probabilities = model.predict(processed_images)

            for probabilities in batch_probabilities:

                predicted_class_id = get_processed_prediction(probabilities, red_confidence)
                matrix[true_class_id, predicted_class_id] += 1

            # Make sums relative to number of samples for the class
            matrix[true_class_id, :] /= len(images)

    return matrix


def main():

    model_dir = "/home/student/CarND-Capstone/ros/src/tl_detector/light_classification/saved_models/"
    model_path = os.path.join(model_dir, "model128.json")
    weight_path = os.path.join(model_dir, "weights128.hdf5")

    model = utilities.get_model(model_path, weight_path)

    data_dir = "/home/student/data_partition/data/bag_dump_loop_with_traffic_light/"
    # data_dir = "/home/student/data_partition/data/bag_dump_just_traffic_light/"

    red_images = utilities.get_images_at_path(os.path.join(data_dir, "red"))
    yellow_images = utilities.get_images_at_path(os.path.join(data_dir, "yellow"))
    green_images = utilities.get_images_at_path(os.path.join(data_dir, "green"))
    other_images = utilities.get_images_at_path(os.path.join(data_dir, "nolight")) + \
                   utilities.get_images_at_path(os.path.join(data_dir, "unidentified"))

    images_map = {0: red_images, 1: yellow_images, 2: green_images, 3: other_images}

    relative_margins = np.arange(0, 0.5, 0.05)
    red_confidence_thresholds = np.arange(0.4, 1.1, 0.1)

    results = []

    for relative_margin in tqdm.tqdm(relative_margins):

        cropped_images_map = {}

        for class_id, images in images_map.items():
            cropped_images = [crop_image_with_relative_margin(image, relative_margin) for image in images]
            cropped_images_map[class_id] = cropped_images

        for red_confidence_threshold in red_confidence_thresholds:

            matrix = get_confusion_matrix(model, cropped_images_map, red_confidence=red_confidence_threshold)
            results.append((relative_margin, red_confidence_threshold, matrix))

    sorted_results = sorted(results, key=lambda x: np.sum(np.diagonal(x[2])), reverse=True)

    print("Dataset: {}".format(data_dir))
    print("Best result:")

    best_result = sorted_results[0]
    relative_margin = best_result[0]
    red_confidence_threshold = best_result[1]
    matrix = best_result[2]

    print("Relative margin: {}, red confidence threshold: {}".format(relative_margin, red_confidence_threshold))
    print("Order: red, yellow, green, others")
    print(matrix)


if __name__ == "__main__":

    main()
