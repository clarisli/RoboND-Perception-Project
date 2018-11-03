#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, yaml_dict_list):
    data_dict = {"object_list": yaml_dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Function to search list of dictionaries and return a selected value in selected dictionary
def search_dictionaries(key1, value1, key2, list_of_dictionaries):
    selected_dic = [element for element in list_of_dictionaries if element[key1] == value1][0]
    selected_val = selected_dic.get(key2)
    return selected_val

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    
	# Convert a ROS PointCloud2 message to a pcl PointXYZRGB
	cloud = ros_to_pcl(pcl_msg)

	# Statistical Outlier Filtering-
	# Much like the previous filters, we start by creating a filter object: 
	outlier_filter = cloud.make_statistical_outlier_filter()
	# Set the number of neighboring points to analyze for any given point
	outlier_filter.set_mean_k(3)
	# Set threshold scale factor
	x = 0.3
	# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
	outlier_filter.set_std_dev_mul_thresh(x)
	# Finally call the filter function for magic
	cloud_filtered = outlier_filter.filter()

    	# Voxel Grid Downsampling
    	vox = cloud_filtered.make_voxel_grid_filter()
    	LEAF_SIZE = 0.005
    	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    	cloud_filtered = vox.filter()

	# Create a PassThrough filter object (z axis)
	passthrough = cloud_filtered.make_passthrough_filter()
	filter_axis = 'z'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = 0.6
	axis_max = 1.1
	passthrough.set_filter_limits(axis_min, axis_max)
	cloud_filtered = passthrough.filter()
	# Create a PassThrough filter object (y axis)
	passthrough = cloud_filtered.make_passthrough_filter()
	filter_axis = 'y'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = -0.45
	axis_max = 0.45
	passthrough.set_filter_limits(axis_min, axis_max)
	cloud_filtered = passthrough.filter()
	# Create a PassThrough filter object (x axis)
	passthrough = cloud_filtered.make_passthrough_filter()
	filter_axis = 'x'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = 0.34
	axis_max = 1.0
	passthrough.set_filter_limits(axis_min, axis_max)
	cloud_filtered = passthrough.filter()

    	# RANSAC Plane Segmentation
    	seg = cloud_filtered.make_segmenter()
    	seg.set_model_type(pcl.SACMODEL_PLANE)
    	seg.set_method_type(pcl.SAC_RANSAC)
    	max_distance = 0.01
    	seg.set_distance_threshold(max_distance)
    	inliers, coefficients = seg.segment()					

    	# Extract inliers and outliers
    	extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    	extracted_outliers = cloud_filtered.extract(inliers, negative=True)

   	# Create Cluster-Mask Point Cloud to visualize each cluster separately
    	white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    	tree = white_cloud.make_kdtree()
    	# Create a cluster extraction object
    	ec = white_cloud.make_EuclideanClusterExtraction()
    	# Set tolerances for distance threshold 
    	# as well as minimum and maximum cluster size (in points)
    	ec.set_ClusterTolerance(0.01)
    	ec.set_MinClusterSize(10)
    	ec.set_MaxClusterSize(3000)
    	# Search the k-d tree for clusters
    	ec.set_SearchMethod(tree)
    	# Extract indices for each of the discovered clusters
    	cluster_indices = ec.Extract()

    	#Assign a color corresponding to each segmented object in scene
    	cluster_color = get_color_list(len(cluster_indices))

    	color_cluster_point_list = []

    	for j, indices in enumerate(cluster_indices):
        	for i, indice in enumerate(indices):
            		color_cluster_point_list.append([white_cloud[indice][0],
                                            	white_cloud[indice][1],
                                            	white_cloud[indice][2],
                                             	rgb_to_float(cluster_color[j])])

    	#Create new cloud containing all clusters, each with unique color
    	cluster_cloud = pcl.PointCloud_PointXYZRGB()
    	cluster_cloud.from_list(color_cluster_point_list)


    	# Convert PCL data to ROS messages
    	table_pcl_msg = pcl_to_ros(extracted_inliers)
    	objects_pcl_msg = pcl_to_ros(extracted_outliers)
    	ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    	# Publish ROS messages
    	pcl_objects_pub.publish(objects_pcl_msg)
    	pcl_table_pub.publish(table_pcl_msg)
    	pcl_clusters_pub.publish(ros_cluster_cloud) 


# Exercise-3 TODOs:

    	# Classify the clusters! (loop through each detected cluster one at a time)
    	detected_objects_labels = []
    	detected_objects = []

    	for index, pts_list in enumerate(cluster_indices):
        	# Grab the points for the cluster
        	pcl_cluster = extracted_outliers.extract(pts_list)
        	ros_cluster = pcl_to_ros(pcl_cluster)
        
        	# Compute the associated feature vector
        	chists = compute_color_histograms(ros_cluster, using_hsv=True)
        	normals = get_normals(ros_cluster)
        	nhists = compute_normal_histograms(normals)
        	feature = np.concatenate((chists, nhists))
        
        	# Make the prediction
        	prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        	label = encoder.inverse_transform(prediction)[0]
        	detected_objects_labels.append(label)
        	# Publish a label into RViz
        	label_pos = list(white_cloud[pts_list[0]])
        	label_pos[2] += .2
        	object_markers_pub.publish(make_label(label,label_pos, index))
       		# Add the detected object to the list of detected objects.
        	do = DetectedObject()
        	do.label = label
        	do.cloud = ros_cluster
        	detected_objects.append(do)
    
    	# Publish the list of detected objects
    	detected_objects_pub.publish(detected_objects)

	# Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    	# Could add some logic to determine whether or not your object detections are robust
    	# before calling pr2_mover()
    	try:
        	pr2_mover(detected_objects)
    	except rospy.ROSInterruptException:
       		pass

# Function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    test_scene_num = Int32()
    object_name    = String()
    pick_pose      = Pose()
    place_pose     = Pose()
    arm_name       = String()
    test_scene_num.data = 1
    yaml_dict_list = []

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    # Parse parameters into individual variables
    dropbox_position =	{
	  dropbox_param[0]['group']: dropbox_param[0]['position'],
	  dropbox_param[1]['group']: dropbox_param[1]['position']
    }
    dropbox_name = {
	  dropbox_param[0]['group']: dropbox_param[0]['name'],
	  dropbox_param[1]['group']: dropbox_param[1]['name']
    }
    red_dropbox_position = dropbox_param[0]['position']
    green_dropbox_position = dropbox_param[1]['position']
    
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Calculate detected objects centroids.
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    # Loop through the pick list
    for i in range(0, len(object_list_param)):
        
        # Read object name and group from object list.
        object_name.data = object_list_param[i]['name' ]
        object_group     = object_list_param[i]['group']

        # Select pick pose
        try:
            index = labels.index(object_name.data)
        except ValueError:
            print "Object not detected: %s" %object_name.data
            continue

        pick_pose.position.x = np.asscalar(centroids[index][0])
        pick_pose.position.y = np.asscalar(centroids[index][1])
        pick_pose.position.z = np.asscalar(centroids[index][2])

        # Select place pose
        position = dropbox_position[object_group]
        place_pose.position.x = position[0]
        place_pose.position.y = position[1]
        place_pose.position.z = position[2]

        # Select the arm to be used for pick_place
        arm_name.data = dropbox_name[object_group]

        # Create a list of dictionaries for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        yaml_dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
            print ("Response: ",resp.success)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    yaml_filename = 'output_'+str(test_scene_num.data)+'.yaml'
    send_to_yaml(yaml_filename, yaml_dict_list)

    return

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=False)
    
    # Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)
    
    # Create Publishers
    object_markers_pub = rospy.Publisher("/object_markers",Marker,queue_size =1)
    detected_objects_pub = rospy.Publisher("/detected_objects",DetectedObjectsArray,queue_size =1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("/pcl_clusters", PointCloud2, queue_size=1)
    pr2_base_mover_pub   = rospy.Publisher("/pr2/world_joint_controller/command", Float64, queue_size=10)
    
    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
