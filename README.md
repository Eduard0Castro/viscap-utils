# Viscap Utils

![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)

![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)

Viscap Utils is a repository to support drone operations, object detection, and examples of sensor usage in autonomous drone projects. It includes tools such as ROS2 (Robot Operating System) for motion and parallel processing, OpenCV (Open Computer Vision) for image processing, YOLO (You Only Look Once) for implementing object detection using custom models, cflib (Crazyflie Python Library) for studying and controlling Crazyflie drones developed by Bitcraze.

The viscap_utils subdirectory is a ROS package, meaning the repository must be cloned into the src folder of a ROS2 workspace.

## Table of Contents
- [Installation](#installation)
- [Features](#features)
    - [AIDeck](#aideck)  
    - [ViscapCrazyflie](#viscapcrazyflie)
    - [ESPCAM](#espcam)
    - [Examples](#examples)
    - [Yolo](#yolo)
- [Usage](#usage)
- [Package Structure](#package-structure)
- [Contribution](#contribution)
- [License](#license)

## Installation

Before installing dependencies, ensure that the system supports ROS2 and that a workspace exists with the necessary build tools.

Clone the repository into a ROS2 workspace (`ros2_ws/src`) and install the pip dependencies via:

```sh
pip install -r requirements.txt
```

## Features
### Crazyflie

#### AIDeck
`viscap_utils/viscap_utils/crazyflie/ai_deck.py`

AIDeck allows the creation of an AIDeck object and provides methods to configure a connection via Wi-Fi created by the active deck. This connection enables access to images captured by the camera attached to the AI-deck.

To use it, check the documentation and tutorial available at [ai-deck-tutorial](https://www.bitcraze.io/documentation/tutorials/getting-started-with-aideck/). The method implementations are based on the example available in [opencv-viewer](https://github.com/bitcraze/aideck-gap8-examples/blob/master/examples/other/wifi-img-streamer/opencv-viewer.py).


#### ViscapCrazyflie
`viscap_utils/viscap_utils/crazyflie/viscap_crazyflie.py`

ViscapCrazyflie allows the instantiation of objects to control Crazyflie drones from the Swedish company Bitcraze. Within the class, all connection links with the drones are built, making it possible to initialize tools and auxiliary decks using methods that configure all necessary communication between the machine and the drone through the Crazyradio. Everything operates using ROS2, as the class implements its methods using resources native to a ROS node. Additionally, it integrates with prebuilt classes from cflib (Crazyflie Python Library), which already provides methods for initializing decks such as the Multiranger Deck and also includes the AIDeck class available in this project.

For more information about the functionalities available in cflib, check the complete Bitcraze documentation at  [bitcraze.io](https://www.bitcraze.io/documentation/start/) and also in the [GitHub repository](https://github.com/bitcraze/crazyflie-lib-python/tree/master), where there are coding examples used as a basis for developing the applications present here.presentes. 

### ESPCAM

`viscap_utils/viscap_utils/espcam/`

Implementation of video examples and aruco detection using an ESP32-CAM. The steps to configure this device along with an ESP32-CAM-MB can be accessed through the video: [Getting Started Programming ESP32-CAM With ESP32-CAM-MB Micro USB Programmer Serial Converter Loader](https://www.youtube.com/watch?v=z67mfL63e2M&t=319s).

### Examples
The `viscap_utils/viscap_utils/examples/` directory contains example scripts for Crazyflie applications:
- **`multiranger_test.py`**: Test with the lidars present in the multiranger deck.
- **`square.py`**: Square trajectory movement with a single Crazyflie without using ROS.
- **`swarm_test.py`**: Control of a drone swarm using ViscapCrazyflie methods.
- **`two_crazyflies.py`**: Swarm test without ROS.

### Yolo

Implement Object Detect class to apply an YOLO model detection. It works with ultralytics library for load the model and process the detection and OpenCV to draw the bounding boxes and place the infos in the image.

You can use this application in two ways. The first is to only use this repository. The media and models folders can be used to store the media where you want to detect the object and the customized model. The outputs will be stored in the default folders that were explained in the class methods. The second is use in another script that needs of this resource and you can import the class and use the methods by passing your own paths. 

After colcon build in the workspace, you can import in another python script like:

```python
from viscap_utils.yolo.object_detect import ObjectDetect
```

## Usage

- Example to import from the package root to desired module:
```python
from viscap_utils.crazyflie.ai_deck import AIDeck
```

- Run with Python3 inside the script directory (examples):
```sh
python3 multiranger_test.py
```

Or using ROS:

```sh
ros2 run viscap_utils multiranger_test  
```
## Package Structure
```
viscap-utils/
│── viscap_utils/
│   │── __init__.py
│   │── package.xml
│   │── setup.py
|   |── crazyflie/
|   |   |── ai_deck.py
|   |   |── viscap_crazyflie.py
|   |── espcam/
|   |   |── aruco_detect.py
|   |   |── video_espcam.py
│   │── examples/
│   │   │── multiranger_test.py
│   │   │── square.py
│   │   │── swarm_test.py
│   │   │── two_crazyflies.py
|   |── yolo/
|   |   |── media/
|   |   |   |── images/
|   |   |   |   |── input/
|   |   |   |   |── output/
|   |   |   |── videos/
|   |   |── models/
|   |   |── object_detect.py

```

## Contribution

Feel free to contribute by adding new features or refactoring the code in this repository.

## License

This project is licensed under the Apache License 2.0. See the LICENSE file for more details.

