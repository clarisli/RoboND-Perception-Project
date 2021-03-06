## Project: Perception Pick & Place


The goal of this project is to implement a perception pipeline, correctly identify the majority of objects in three different scenarios, and complete a tabletop pick and place operation using PR2.

1. Extract features and train an SVM model on new objects. 
2. Write a ROS node and subscribe to `/pr2/world/points` topic.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  

[//]: # (Image References)

[image1]: ./misc_images/statistical_outlier_filtering.png
[image2]: ./misc_images/original.png
[image3]: ./misc_images/clustering.png
[image4]: ./misc_images/objects.png
[image5]: ./misc_images/table.png
[image6]: ./misc_images/passthrough_before.png
[image7]: ./misc_images/passthrough_z.png
[image8]: ./misc_images/passthrough_y.png
[image9]: ./misc_images/passthrough_x.png
[image10]: ./misc_images/vox.png
[image11]: ./misc_images/test1.png
[image12]: ./misc_images/test2.png
[image13]: ./misc_images/test3.png
[image14]: ./misc_images/confusion.png

## Project Setup
For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Perception-Project.git
```
### Note: If you have the Kinematics Pick and Place project in the same ROS Workspace as this project, please remove the 'gazebo_grasp_plugin' directory from the `RoboND-Perception-Project/` directory otherwise ignore this note. 

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```
Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/models:$GAZEBO_MODEL_PATH
```

If you haven’t already, following line can be added to your .bashrc to auto-source all new terminals
```
source ~/catkin_ws/devel/setup.bash
```

To run the demo:
```sh
$ cd ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts
$ chmod u+x pr2_safe_spawner.sh
$ ./pr2_safe_spawner.sh
```
![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)



Once Gazebo is up and running, make sure you see following in the gazebo world:
- Robot

- Table arrangement

- Three target objects on the table

- Dropboxes on either sides of the robot


If any of these items are missing, please report as an issue on [the waffle board](https://waffle.io/udacity/robotics-nanodegree-issues).

In your RViz window, you should see the robot and a partial collision map displayed:

![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Proceed through the demo by pressing the ‘Next’ button on the RViz window when a prompt appears in your active terminal

The demo ends when the robot has successfully picked and placed all objects into respective dropboxes (though sometimes the robot gets excited and throws objects across the room!)

Close all active terminal windows using **ctrl+c** before restarting the demo.

You can launch the project scenario like this:
```sh
$ roslaunch pr2_robot pick_place_project.launch
```

---

Note: if you're getting an error when running the statistical outlier filter like this:

```
Error: TypeError: __cinit__() takes exactly 1 positional argument (0 given)
```

Install `python-pcl` as following:

Install cython:

```
$ sudo pip install cython
```

Build and Install pcl-python:

```
$ cd ~/RoboND-Perception-Project/python-pcl
$ python setup.py build
$ sudo python setup.py install
```

Install pcl-tools:

```
$ sudo apt-get install pcl-tools
```



## Perception Pipeline

### 1. Calibration, Fitlering, and Segmentation

#### 1.1 Statistical Outlier Filtering

I first applied PCL's StatisticalOutlierRemoval filter to remove noise. For each point in the point cloud, it computes the distance to all of its neighbors, and then calculates a mean distance. All points whose mean distances are outside of an interval are considered to be outliers and removed from the point cloud.

I did this in lines 61 to 71 in `project.py`.

| Original                 |  Filtered                | 
:-------------------------:|:-------------------------:|
|![alt text][image2]       | ![alt text][image1]     | 

#### 1.2 Voxel Grid Downsampling

To save the comptational resource, I downsampled the point cloud with VoxelGrid Downsampling Filter. The goal was to derive a point cloud that has fewer points but should still do a good job of representing the input point cloud as a whole.

![alt text][image10]

#### 1.3 Pass Through Filtering

To further remove useless data from the point cloud, I applied Pass Through Filter to define the region of interest.

| Original                 | Passthrough z-axis      | 
:-------------------------:|:-----------------------:|
|![alt text][image6]       | ![alt text][image7]     | 
| Passthrough y-axis       | Passthrough x-axis      | 
|![alt text][image8]       | ![alt text][image9]     | 

#### 1.4 RANSAC Plane Fitting

I then used RANSAC plane fitting to remove the table from the scene. The RANSAC algorithm assumes that all of the data in a dataset is composed of both inliers and outliers, where inliers can be defined by a particular model with a specific set of parameters, while outliers do not fit that model and hence can be discarded. With a plane model, the inlier is the table, and the outliers are the objects.

| Table                 | Objects      | 
:-------------------------:|:-----------------------:|
|![alt text][image5]       | ![alt text][image4]     | 

#### 2. Clustering for Segmentation

##### 2.1 Euclidean Clustering

With the unwanted data filtered out, the next step is to segment the remaining points into individual objects. I applied Euclidean Clustering to cluster the points closer to each other together. 

I first converted the **XYZRGB** point cloud to **XYZ**, because PCL's Euclidean Clustering algorithm requires a point cloud with only spatial information. Then I constructed a k-d tree from this point cloud and performed the cluster extraction. The results are visualized below:

![alt text][image3]

### 3. Object Recognition

The goal is to reliably locate the things you're looking for, regardless of its position and orientation. The key here to identify the features that best describe that object.

#### 3.1 Feature Extraction

I used the color and shape as the features:

* Color: The color space HSV was used instead of RGB, because RGB is not robust enough - the objects can appear to have different color under different lighting condition. I converted color information into features using histograms. This feature has no dependency on spatial structre, and objects with different poses and orientations will match. 
* Shape: The distribution of surface normals was used to capture different shapes.

The hitograms were normalized to accomodate with the variations in image size.

I did this in `/sensor_stick/src/sensor_stick/features.py` and `/sensor_stick/scripts/capture_features.py`.

#### 3.2 Train the Model

I used the features above to train the model with Support Vector Machine (SVM) and obtained an accuracy of 98% - I tuned the number of orientations to 128 and number of histogram bins to 64. Here's the confusion chart:

![alt text][image14]

I did this in `/sensor_stick/scripts/train_svm.py`.


## Pick and Place Setup

With the detected objects from above, I composed the necessary messages to send to the `pick_place_routine` service.

The ground truth pick lists, i.e., objects to be collected can be found at the `pick_list_*.yaml` files under `/pr2_robot/config`. For example, `pick_list_1.yaml` file looks like this:

```
object_list:
  - name: biscuits
    group: green
  - name: soap
    group: green
  - name: soap2
    group: red
```

By verifying each item in the pick list with detected objects, I only proceed to pick and place the accurately identified items.

The pick and place operation is implemented as a request-response system, the format of the service is defined in `PickPlace.srv` under `pr2_robot/srv`:


Name           |  Message Type       | Description
:-------------:|:-------------------:|:-------------------|
test_scene_num | `std_msgs/Int32`.   | The current test scene
object_name    | `std_msgs/String`.  | Name of the object
arm_name       | `std_msgs/String`.  | Name of the arm
pick_pose      | `geometry_msgs/Pose`| Calculated Pose of recognized object's centroid
place_pose     | `geometry_msgs/Pose`| Object placement Pose

The test_scene_num is the world file name used in `pick_place_project.launch` file:

```
<arg name="world_name" value="$(find pr2_robot)/worlds/test1.world"/>
```  

I obtained the object_name from the pick list.

I retrieved the dropbox's information from `dropbox.yaml`, it looks like this:

```
dropbox:
  - name: left
    group: red
    position: [0,0.71,0.605]
  - name: right
    group: green
    position: [0,-0.71,0.605]
```

Based on the group associated with each object, I assigned the arm_name with the same name of the dropbox under the same group. Then I used the same dropbox's position for place_pose. 

I did this in the function `pr2_mover()` in `/pr_robot/scripts/project.py`.

### Results

For each test scene, I output the request parameters to a `output_*.yaml`, and saved them under the `/pr_robots/scripts/` folder.

#### Scene 1

100% (3/3) objects in test1.world: [output_1.yaml](/pr_robots/scripts/output_1.yaml)

![alt text][image11]

#### Scene 2

100% (5/5) objects in test2.world: [output_2.yaml](/pr_robots/scripts/output_2.yaml)

![alt text][image12]

#### Scene 3

100% (8/8) objects in test3.world: [output_3.yaml](/pr_robots/scripts/output_3.yaml)

![alt text][image13]

### Future Works
* Implement collision mapping
* Use `pick_lace_server` to execute the pick and place operation
