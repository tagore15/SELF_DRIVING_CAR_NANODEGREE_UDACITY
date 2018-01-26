import sys
import os
import cv2
from bstld.read_label_file import get_all_labels
from bstld.show_label_images import ir

def save_tl_images(input_yaml, output_folder,margin = 10):
    """
    Extracts labelled pictures of traffic lights.
    Saves them as separate files in specified output_folder.
    :param input_yaml: Path to yaml file
    :param output_folder: path to folder. created if does not exist
    :param margin:additional pixels to the traffic light
    """
    images = get_all_labels(input_yaml)

    assert output_folder is not None

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    num = 1
    for i, image_dict in enumerate(images):
        image = cv2.imread(image_dict['path'])
        if image is None:
            raise IOError('Could not open image path', image_dict['path'])

        for box in image_dict['boxes']:
            xmin = ir(box['x_min'])
            ymin = ir(box['y_min'])
            xmax = ir(box['x_max'])
            ymax = ir(box['y_max'])
           
            if xmin > xmax:
                xmin, xmax = xmax, xmin
            if ymin > ymax:
                ymin, ymax = ymax, ymin
            label = box['label']
            roi = image[ymin - margin:(ymax + 1) + margin, xmin - margin:(xmax + 1) + margin]
            filename = os.path.join(output_folder,
                                    str(num).zfill(6) + '_' + label.lower() + '.png')
            cv2.imwrite(filename, roi)
            if os.stat(filename).st_size==0:
                os.remove(filename)
                print("Saved file size is zero, deleting: {} {}".format(i, filename))
            num += 1

if __name__ == '__main__':
    save_tl_images("data/test.yaml", output_folder="data/cropped_data/test_margin10", margin = 10)
    save_tl_images("data/additional_train.yaml", output_folder="data/cropped_data/additional_train_margin10", margin = 10)
    save_tl_images("data/train.yaml", output_folder="data/cropped_data/train_margin10", margin = 10)
    
