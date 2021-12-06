# Dataset rosbags for Viewpoint-tolerant Semantic Segmentation

## Description

This Readme.md will introduce the training and test datasets used in the paper "Viewpoint-tolerant Semamtic Segmentation in Aerial Logistics". Pleas follow this [link](https://drive.google.com/drive/folders/1nPrOjodKeOEPm9Dvwt7ekm2NHuoDxGBF?usp=sharing) to download the ziped dataset bags. All the datasets are given in the rosbags and collected from the custom-made OpenGL-based drone simulator. 

Data is sampled over 5 models: **hospital**, **garden**, **warehouse**, **house garden** and **skyscrapres**. The technical details will be given in the `Dataset` part. Each model contains respectively training dataset, named `circle`, and test datasets, named `star`. Please refer the original paper for the detailed sampling approach. Data is sampled at different viewpoints above the model and at each viewpoint 4 different kinds of information are collected:
+ rgb image, in ros topic `/fireflyvrglasses_for_robots_ros/color_map`, `.png` format with resolution of `480x752`
+ semantic annotation, in ros topic `/firefly/vrglasses_for_robots_ros/semantic_map`, `.png` format with resolution of `480x752`
+ depth information, in ros topic `/firefly/vrglasses_for_robots_ros/depth_map `, `.exr` format with resolution of `480x752`
+ camera position information, in ros topic `/firefly/vrglasses_for_robots_ros/camera_odometry_out`, `.csv` format, contains timestamp, positions (`x`,`y`,`z`) and orientations (`x`,`y`,`z`, `w`)

## Semantic Class Definition
| Class          | Palette                                                             |             Color Example             | Description                                                                                  |
|----------------|---------------------------------------------------------------------|:-------------------------------------:|----------------------------------------------------------------------------------------------|
| **Pavement**   | <div style="width: 170pt"> RGB: (81, 0, 81)<br/>HEX: #510051 </div> |  ![](./resources/color_pavement.png)  | Man-made flat ground, asphalt, sidewalk, which are easy to traverse by all wheeled vehicles. |
| **Terrian**    | RGB: (152, 251, 152)<br/>HEX: #98FB98                               |  ![](./resources/color_terrian.png)   | Bare terrain, grass fields, mud, which can be traversed by off-road wheeled vehicles.        |
| **Water**      | RGB: (150, 170, 250)<br/>HEX: #96AAFA                               |   ![](./resources/color_water.png)    | Lake, sea, rivers, swimming pool and so on.                                                  |
| **Sky**        | RGB: (70, 130, 180)<br/>HEX: #4682B4                                |    ![](./resources/color_sky.png)     | Sky.                                                                                         |
| **Building**   | RGB: (70, 70, 70)<br/>HEX: #464646                                  |  ![](./resources/color_building.png)  | All types of edification, including the windows, doors, and rooftops.                        |
| **Vegetation** | RGB: (107, 142, 35)<br/>HEX: #6A8E23                                | ![](./resources/color_vegetation.png) | Trees, bushes, plants, which are not easy to traverse by any wheeled vehicle.                |
| **Person**     | RGB: (220,20,60)<br/>HEX: #DC143C                                   |   ![](./resources/color_person.png)   | Pedestrians.                                                                                 |
| **Rider**      | RGB: (255,0,0)<br/>HEX: #FF000                                      |   ![](./resources/color_riders.png)   | Bicyclists and motorcyclists                                                                 |
| **Vehicle**    | RGB: (0,0,142)<br/>HEX: #0008E                                      |  ![](./resources/color_vehicle.png)   | Car, truck, train, boat and so on.                                                           |
| **Others & Unlabeled**    | RGB: (0,0,0)<br/>HEX: #00000                                        |   ![](./resources/color_others.png)   | Elements that do not fit any other class or areas without a model, such as trash bins, background, or failures on the model.   |

## Dataset

### Hospital

<div style="text-align: center;">
    <img src="https://drive.google.com/uc?id=1X7kRBN5lTNsEtw9z1VhVc_U-SF5UR5fP">
    Multi views of hospital model
</div>

+ **circle**:   
    * **sampling position**: height 90 m, pitch angle -60°
    * **number of images**: 288
+ **star**:
    * **number of images in each sub-dataset**: 432
### Warehouse

<div style="text-align: center;">
    <img src="https://drive.google.com/uc?id=1vSMVKEo7ZhFNKdgQGiTckumZ4-PcH2p7">
    Multi views of rehouse model
</div>

+ **circle**:   
    * **sampling position**: height 40 m, pitch angle -60°
    * **number of images**: 216
+ **star**:
    * **number of images in each sub-dataset**: 225

### Garden
<div style="text-align: center;">
    <img src="https://drive.google.com/uc?id=1o0CPzvpYe893KeJeVG6Iax7IIRxDL7vU">
    Multi views of garden model
</div>

+ **circle**:   
    * **sampling position**: height 60 m, pitch angle -60°
    * **number of images**: 216
+ **star**:
    * **number of images in each sub-dataset**: 225

### House Garden
<div style="text-align: center;">
    <img src="https://drive.google.com/uc?id=1f2bAvbpMNmRixs3PKreT_tpMspVrig8x">
    Multi views of house garden model
</div>

+ **circle**:   
    * **sampling position**: height 40 m, pitch angle -60°
    * **number of images**: 288
+ **star**:
    * **number of images in each sub-dataset**: 225
  
### Skyscrapers

<div style="text-align: center;">
    <img src="https://drive.google.com/uc?id=1Pm44d8o3x-6NfLAzPw2FYNMFdoa7Vvdi">
    Multi views of house garden model
</div>

+ **circle**:   
    * **sampling position**: height 90 m, pitch angle -60°
    * **number of images**: 288
+ **star**:
    * **number of images in each sub-dataset**: 480

## Usage

extract data with [`vi_bag_tools`](https://github.com/VIS4ROB-lab/vi_bag_tools)

```sh
  $ roscore
  $ cd ~/catkin_ws/
  $ source ./devel/setup.bash
  $ bagextractor_asl_format --image-topics /firefly/vrglasses_for_robots_ros/color_map --depth_map_topics /firefly/vrglasses_for_robots_ros/depth_map --semantic-topics /firefly/vrglasses_for_robots_ros/semantic_map --gt_odometry_topics /firefly/vi_sensor/ground_truth/odometry /firefly/vrglasses_for_robots_ros/camera_odometry_out --bag /media/2020-10-27-18-43-43.bag --output-folder /home/lucas/data/output_dir
```

The extracted data has the following file structure:  
```
.
├── ann_dir_1            # annotated semantic maps
├── image_dir            # rgb images
├── depth0             
│   └── data             # depth information
└── gt_odometry0  
    └── data.csv         # pose information
```
In all the `circle` sampled data, there are always 7 or 8 more same images (or pose information) at the beginning. To resist the image lost in the rosbag recording process, we sampled 10 more additional fixed-position images at the beginning. After extraction, 7 or 8 of them will remain. They should be deleted in all the topics before training.
