
## Traffic Lights Classification



## Setup

The Bosch Smsll Traffic Lights Dataset is available from [here](https://hci.iwr.uni-heidelberg.de/node/6132).

* Please download rgb files
* Unzip the files and put the files in the data folder(The dataset has around 20GB)
* The structure of data folder is following:

```

data
|
|----rgb
|     |
|     |---additional
|     |       |
|     |       |-2015-10-05-10-52-01_bag
|     |       |   |
|     |       ... | - 24594.png
|     |       |   | - 24664.png
|     |       |   | - 24734.png
|     |       |-2015-10-05-55-33_bag
|     |	      ...
|     |
|     |
|     |---train
|     |       |
|     |       |-2015-05-29-15-29-39_arastradero_traffic_light_loop_bag
|     |       |
|     |       |-2015-10-05-10-52-01_bag
|     |       ...
|     |
|     |---test
|
|
|-----additional_train.yaml
|
|
|-----train.yaml
|
|
|-----test.yaml
|
|
|
|----udacity_data
|     |
|     |--bag_dump_just_traffic_light
|     |     |
|     |     |- green
      |     |- nolight
      |     |- red
      |     |- unidentified
      |     |- yellow
      |
      |--bag_dump_loop_with_traffic_light
            |
	    |- green
            |- nolight
            |- red
            |- unidentified
            |- yellow

```

You can extract traffic light images and save the images by doing following.(You can change the margin size and output_folder name as you like)

```bash
python crop_traffic_lights.py 
```

After extracting traffic light images, you can train traffic light classifier at
```
tl_classifier_cnn.ipynb
```
