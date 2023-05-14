#!/usr/bin/env python3


import os
import cv2
import tf2_ros
import numpy as np
import math
import importlib.util
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point,Pose
from tf2_geometry_msgs import PoseStamped
from image_geometry import PinholeCameraModel
from nav_msgs.msg import Odometry, OccupancyGrid
from robotics_challenge.msg import Ball, SnapshotBalls


rospy.init_node('ML_Detector_node')


#Constants
BALL_SIZE = 0.055
LINE_X_POSITION = 0.02 #To deal with the inacurracy of the coordinates retrieved from camera to 3d point

CAMERA_TOPIC = '/camera/image'
CAMERA_INFO_TOPIC = '/camera/camera_info'


class BallDetector:

    def __init__(self):
        print('-------------------------------DETECTOR IS UP-------------------------------')
        # Initialize the CvBridge object
        self.bridge = CvBridge()

        #Odom
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.robot_pos_callback)
        self.robot_pos = Pose()


        #Setup PinholeCamera model/ Image topic
        self.sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.callback)
        self.camera_info_sub = rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo, self.update_camera_info)
        self.camera = PinholeCameraModel()
        #self.camera.fromCameraInfo(self.camera_info)
        print(' Camera Connected Succesfully')

        #Frame transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ball_pub = rospy.Publisher('/balls_location', Ball, queue_size=1)
        self.ball_blob_pub = rospy.Publisher('/balls_location/blob', SnapshotBalls, queue_size=1)
        self.output_mask_pub=rospy.Publisher('/output_mask', Image, queue_size=3)

        #Map
        #self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        #self.map_pub = rospy.Publisher('/modified_map', OccupancyGrid, queue_size=1)

        #setup Deep learning model
        # Parse user inputs
        MODEL_NAME = '/home/mohamed/custom_model_lite/'
        GRAPH_NAME = 'detect.tflite'
        LABELMAP_NAME = 'labelmap.txt'

        self.min_conf_threshold = 0.5


        # Import TensorFlow libraries
        # If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
        # If using Coral Edge TPU, import the load_delegate library
        pkg = importlib.util.find_spec('tflite_runtime')
        if pkg:
            from tflite_runtime.interpreter import Interpreter



        # Get path to current working directory
        self.CWD_PATH = os.getcwd()

        # Path to .tflite file, which contains the model that is used for object detection
        self.PATH_TO_CKPT = os.path.join(self.CWD_PATH,MODEL_NAME,GRAPH_NAME)

        # Path to label map file
        self.PATH_TO_LABELS = os.path.join(self.CWD_PATH,MODEL_NAME,LABELMAP_NAME)

        # Load the label map
        with open(self.PATH_TO_LABELS, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]

        # Have to do a weird fix for label map if using the COCO "starter model" from
        # https://www.tensorflow.org/lite/models/object_detection/overview
        # First label is '???', which has to be removed.
        if self.labels[0] == '???':
            del(self.labels[0])

        # Load the Tensorflow Lite model.
        # If using Edge TPU, use special load_delegate argument
        self.interpreter = Interpreter(model_path=self.PATH_TO_CKPT)

        self.interpreter.allocate_tensors()

        # Get model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

        self.input_mean = 127.5
        self.input_std = 127.5

        # Check output layer name to determine if this model was created with TF2 or TF1,
        # because outputs are ordered differently for TF2 and TF1 models
        outname = self.output_details[0]['name']

        if ('StatefulPartitionedCall' in outname): # This is a TF2 model
            self.boxes_idx, self.classes_idx, self.scores_idx = 1, 3, 0
        else: # This is a TF1 model
            self.boxes_idx, self.classes_idx, self.scores_idx = 0, 1, 2
    def update_camera_info(self, msg):
        self.camera.fromCameraInfo(msg)


    def callback(self, img):
        bridge = CvBridge()

        # Load image and resize to expected shape [1xHxWx3]
        image = bridge.imgmsg_to_cv2(img)
        #self.camera.rectifyImage(image, image)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        imH, imW, _ = image.shape 
        image_resized = cv2.resize(image, (self.width, self.height))
        input_data = np.expand_dims(image_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.floating_model:
            input_data = (np.float32(input_data) - self.input_mean) / self.input_std

        # Perform the actual detection by running the model with the image as input
        self.interpreter.set_tensor(self.input_details[0]['index'],input_data)
        self.interpreter.invoke()

        # Retrieve detection results
        boxes = self.interpreter.get_tensor(self.output_details[self.boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        classes = self.interpreter.get_tensor(self.output_details[self.classes_idx]['index'])[0] # Class index of detected objects
        scores = self.interpreter.get_tensor(self.output_details[self.scores_idx]['index'])[0] # Confidence of detected objects

        detections = SnapshotBalls()

        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > self.min_conf_threshold) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * imH)))
                xmin = int(max(1,(boxes[i][1] * imW)))
                ymax = int(min(imH,(boxes[i][2] * imH)))
                xmax = int(min(imW,(boxes[i][3] * imW)))

                #print(xmin,xmax, ymin, ymax)
                #print((ymax-ymin)/2)
                radius = round((ymax-ymin)/2)
                size = (xmax-xmin) * (ymax-ymin)
                #print(size)
                cx = (xmax+xmin)/2
                cy = (ymax+ymin)/2

                cv2.rectangle(image, (xmin,ymin), (xmax,ymax), (0, 0, 255), 2)
                #cv2.circle(image, (cx,cy), radius, (10, 255, 0), 1)
                #cv2.circle(image, (cx,cy), radius=0, color=(0, 0, 255), thickness=-1)

                # Draw label
                object_name = self.labels[int(classes[i])] # Look up object name from "labels" array using class index
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(image, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                cv2.putText(image, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

                #detections.append([object_name, scores[i], xmin, ymin, xmax, ymax])
                
                print(object_name)
                position3d = self.find_3d_point(cx,cy,size, 'odom')
                print("3d Pose : \n", position3d)
                #print(position3d)
                #if position3d.x <= LINE_X_POSITION:
                print('considered')
                ball= Ball(position3d=self.find_3d_point(cx,cy,size, 'base_link'), positionPixel=Point(cx,cy,self.calculate_distance(size)), close_ball=Point(xmax-xmin, ymax-ymin, 0))
                detections.list.append(ball)
            

        if detections.list:
            nearest_ball = max(detections.list, key=lambda ball : ball.positionPixel.z)
            detections.list.insert(0, detections.list.pop(detections.list.index(nearest_ball)))
            self.ball_blob_pub.publish(detections)
        #Incase there is no balls, we still send an empty array
        else:
            self.ball_blob_pub.publish(SnapshotBalls())
            print("No balls detected.")
        
        cv2.imshow('Object detector', image)
        cv2.waitKey(1)

    #Odom Subscriber callback
    def robot_pos_callback(self, msg):
        self.robot_pos = msg.pose.pose
    def map_callback(self, map):
        self.map = map

        #self.update_map()

    def calculate_distance(self, ball_size_pixels):
        #Preparing variables
        fx = self.camera.fx()
        fy = self.camera.fy()
        print(fx,fy)
        distance = math.sqrt(((BALL_SIZE * fx)*(BALL_SIZE * fx)) / ball_size_pixels)
        return distance
    def find_3d_point(self, px,py,ball_radius, frame):
        distance = self.calculate_distance(ball_radius)
        #distance = distance +0.1
        #distance_y = distance_y +0.1

        #Convert to 3D ray
        point = self.camera.projectPixelTo3dRay((px,py))
        print(point)

        theta = math.atan(distance)
        print(math.degrees(theta))
        distance_x = math.cos(theta)
        distance_y = math.sin(theta)
        print(distance)

        #Convert to 3D Point
        dpoint = Point(point[0] *distance/point[2], point[1]*distance, point[2]*distance/point[2])
        #print(dpoint)

        #Preparing variables for frame transform
        pose = Pose()
        pose.position = dpoint
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.camera.tfFrame()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose

        
        #Preforming transform
        try:
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, frame, rospy.Duration(0.1))
            return output_pose_stamped.pose.position
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Failed !')
        
    


if __name__=='__main__':
    bDetector = BallDetector()

    # Clean up
    cv2.destroyAllWindows()
    rospy.spin()
