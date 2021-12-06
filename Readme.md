# Dataset rosbags for Viewpoint-tolerant Semantic Segmentation

## Description

This Readme.md will introduce the training and test datasets used in the paper "Viewpoint-tolerant Semamtic Segmentation in Aerial Logistics". Pleas follow this [link](https://drive.google.com/drive/folders/1nPrOjodKeOEPm9Dvwt7ekm2NHuoDxGBF?usp=sharing) to download the ziped dataset bags. All the datasets are given in the rosbags and collected from the custom-made OpenGL-based drone simulator. 

Data is sampled over 5 models: **hospital**, **garden**, **warehouse**, **house garden** and **skyscrapres**. The technical details will be given in the `Dataset` part. Each model contains respectively training dataset, named `circle`, and test datasets, named `star`. Please refer the original paper for the detailed sampling approach. Data is sampled at different viewpoints above the model and at each viewpoint 4 different kinds of information are collected:
+ rgb image, in ros topic `/fireflyvrglasses_for_robots_ros/color_map`, `png` format with resolution of `480x752`
+ semantic annotation, in ros topic `/firefly/vrglasses_for_robots_ros/semantic_map`, `png` format with resolution of `480x752`
+ depth information, in ros topic `/firefly/vrglasses_for_robots_ros/depth_map `, `exr` format with resolution of `480x752`
+ camera position information, in ros topic `/firefly/vrglasses_for_robots_ros/camera_odometry_out`, `csv` format, contains timestamp, positions (`x`,`y`,`z`) and orientations (`x`,`y`,`z`, `w`)

## Semantic Class Definition

+ <table><tr><td bgcolor=#510051 width="160" height="30" > <b>Pavement <br>(81,0,81) </b> </td> <td>Man-made flat ground, asphalt, sidewalk, which are easy to traverse by all wheeled vehicles; </td></tr></table>  

+ <table><tr><td bgcolor=#98fb98 width="160" height="30"> <b>Terrian <br>(152,251,152) </b> </td> <td>Bare terrain, grass fields, mud, which can be traversed by off-road wheeled vehicles; </td></tr></table> 
+ <table><tr><td bgcolor=#96aafa width="160" height="30"> <b>Water <br>(150,170,250) </b> </td> <td> Lake, sea, rivers, swimming pool and so on; </td></tr></table> 

+ <table><tr><td bgcolor=#4682b4 width="160" height="30"> <b>Sky <br>(70,130,180) </b> </td> <td> Sky; </td></tr></table> 

+ <table><tr><td bgcolor=#464646 width="160" height="30"> <b>Building <br>(70,70,70) </b> </td> <td> All types of edification, including the windows, doors, and rooftops; </td></tr></table> 


+ <table><tr><td bgcolor=#6a8e23 width="160" height="30"> <b>Vegetation <br>(107,142,35) </b> </td> <td> Trees, bushes, plants, which are not easy to traverse by any wheeled vehicle; </td></tr></table> 

+ <table><tr><td bgcolor=#dc143c width="160" height="30"> <b>Person <br>(220, 20, 60) </b> </td> <td> All the people without transportation, such as pedestrians; </td></tr></table> 

+ <table><tr><td bgcolor=#ff000 width="160" height="30"> <b>Rider <br>(255, 0, 0) </b> </td> <td> All the people without transportation, such as pedestrians; </td></tr></table> 

+ <table><tr><td bgcolor=#0008e width="160" height="30"> <b>Vehicle <br>(0, 0, 142) </b> </td> <td> Car, truck, train, boat, and so on; </td></tr></table> 

+ <table><tr><td bgcolor=#00000 width="160" height="30"> <b>Others & Unlabeled <br> (0, 0, 0) </b> </td> <td> Elements that do not fit any other class or areas without a model, such as trash bins, background, or failures on the model. </td></tr></table> 


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
    * **number of images of each sub-dataset**: 432
### Warehouse

<div style="text-align: center;">
    <img src="https://drive.google.com/uc?id=1vSMVKEo7ZhFNKdgQGiTckumZ4-PcH2p7">
    Multi views of rehouse model
</div>

+ **circle**:   
    * **sampling position**: height 40 m, pitch angle -60°
    * **number of images**: 216
+ **star**:
    * **number of images of each sub-dataset**: 225

### Garden
<div style="text-align: center;">
    <img src="https://drive.google.com/uc?id=1o0CPzvpYe893KeJeVG6Iax7IIRxDL7vU">
    Multi views of garden model
</div>

+ **circle**:   
    * **sampling position**: height 60 m, pitch angle -60°
    * **number of images**: 216
+ **star**:
    * **number of images of each sub-dataset**: 225

### House Garden
<div style="text-align: center;">
    <img src="https://drive.google.com/uc?id=1f2bAvbpMNmRixs3PKreT_tpMspVrig8x">
    Multi views of house garden model
</div>

+ **circle**:   
    * **sampling position**: height 40 m, pitch angle -60°
    * **number of images**: 288
+ **star**:
    * **number of images of each sub-dataset**: 225
  
### Skyscrapers

<div style="text-align: center;">
    <img src="https://drive.google.com/uc?id=1Pm44d8o3x-6NfLAzPw2FYNMFdoa7Vvdi">
    Multi views of house garden model
</div>

+ **circle**:   
    * **sampling position**: height 90 m, pitch angle -60°
    * **number of images**: 288
+ **star**:
    * **number of images of each sub-dataset**: 480

## Usage

extract data with `vi_bag_tools`

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
