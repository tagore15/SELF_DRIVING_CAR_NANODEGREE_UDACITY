"""
Module with various utility functions used by scripts
"""

import glob
import os

import keras
import cv2


def get_images_at_path(dir_path):
    """
    Read all jpg images in provided directory
    :param dir_path:
    :return:
    """

    paths = sorted(glob.glob(os.path.join(dir_path, "*.jpg")))
    return [cv2.imread(path) for path in paths]


def get_model(model_path, weight_path):

    with open(model_path, 'r') as f:
        loaded_model_json = f.read()

    model = keras.models.model_from_json(loaded_model_json)
    model.load_weights(weight_path)
    # self.graph = tf.get_default_graph()

    return model