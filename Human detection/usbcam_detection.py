# Import packages
import os
import argparse
import cv2
import numpy as np
import sys
import time
from threading import Thread
import importlib.util
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from datetime import datetime
from geometry_msgs.msg import Twist


class SecurityPublisher(Node):
    def __init__(self):
        super().__init__("Security_publisher")
        super().__init__("human_publisher")
        self.number_publisher_ = self.create_publisher(Int16, "is_detected", 10)
        self.human_publisher_ = self.create_publisher(Int16,"assist_human", 10)
        self.cmdvel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

class VideoStream:
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self,resolution=(640,480),framerate=30):
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3,resolution[0])
        ret = self.stream.set(4,resolution[1])
            
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update,args=()).start()
        return self


    def update(self):
        while True:
            if self.stopped:
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
	# Return the most recent frame
        return self.frame

    def stop(self):
	# Indicate that the camera and thread should be stopped
        self.stopped = True


parser = argparse.ArgumentParser()
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                    default='detect.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                    default='labelmap.txt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                    default=0.5)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.',
                    default='1280x720')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
                    action='store_true')

args = parser.parse_args()

MODEL_NAME = 'Sample_TFLite_model'
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu

pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
    if use_TPU:
        from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if use_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

# If using Edge TPU, assign filename for Edge TPU model
if use_TPU:
    # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
    if (GRAPH_NAME == 'detect.tflite'):
        GRAPH_NAME = 'edgetpu.tflite'       

# Get path to current working directory
CWD_PATH = os.getcwd()

# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

if labels[0] == '???':
    del(labels[0])

if use_TPU:
    interpreter = Interpreter(model_path=PATH_TO_CKPT,
                              experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
    print(PATH_TO_CKPT)
else:
    interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

rclpy.init()

node = SecurityPublisher()
publish_detected = 1

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

outname = output_details[0]['name']

if ('StatefulPartitionedCall' in outname): # This is a TF2 model
    boxes_idx, classes_idx, scores_idx = 1, 3, 0
else: # This is a TF1 model
    boxes_idx, classes_idx, scores_idx = 0, 1, 2

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

max_area = 640 * 480
threshold = 0.9 * max_area
a_human = 1

KNOWN_DISTANCE = 250.0 #150.0
KNOWN_WIDTH = 70

def focal_length(known_distance, known_width, width_image):
    # focal_length_value = known_distance * (height_image/known_heigth)
    focal_length_value = (width_image * known_distance) / known_width
    return focal_length_value

def distance_finder(focal_length, real_face_width, width_image):
    # distance = focal_length * (real_face_height/height_image)
    distance = (real_face_width * focal_length) / width_image
    return distance

focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, width)

# Initialize video stream
videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
time.sleep(1)

while True:
    t1 = cv2.getTickCount()
    # Grab frame from video stream
    frame1 = videostream.read()
    
    # Acquire frame and resize to expected shape [1xHxWx3]
    frame = frame1.copy()
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (width, height))
    input_data = np.expand_dims(frame_resized, axis=0)
    h, w, d = frame.shape

    person_count = []

    if floating_model:
        input_data = (np.float32(input_data) - input_mean) / input_std

    interpreter.set_tensor(input_details[0]['index'],input_data)
    interpreter.invoke()
    
    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects

    # Loop over all detections and draw detection box if confidence is above minimum threshold
    for i in range(len(scores)):
        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0) and (int(classes[i])) == 0):
            cmd = Int16()
            cmd.data = int(publish_detected)
            node.number_publisher_.publish(cmd)
            node.get_logger().info("Sending human detected"+ str(publish_detected))
            human = Int16()
            human.data = int(a_human)
            vel = Twist()
            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
            ymin = int(max(1,(boxes[i][0] * imH)))
            xmin = int(max(1,(boxes[i][1] * imW)))
            ymax = int(min(imH,(boxes[i][2] * imH)))
            xmax = int(min(imW,(boxes[i][3] * imW)))
            length = xmax - xmin
            breadth = ymax - ymin
            area = length * breadth
            box_val = {'area': area, 'xmax': xmax, 'xmin': xmin, 'ymax':ymax, 'ymin':ymin}
            person_count.append(box_val)            
            if(area > threshold):
                node.human_publisher_.publish(human)
                # vel.linear.x = 0
                # vel.angular.z = 0
                # node.cmdvel_publisher_.publish(vel)
                node.get_logger().info("Sending assist human: " + str(human))
            # else:
            #     if (len(person_count) == 1):
            #         cx = int((xmax+xmin)/2)
            #         cy = int((ymin+ymax)/2)    
            #         err = cx - w/2
            #         vel.linear.x = 0.16
            #         vel.angular.z = -float(err) / 400
            #         node.cmdvel_publisher_.publish(vel)
            #         node.get_logger().info("Sending velocity: " + str(vel.angular.z))

            #     elif (len(person_count) > 1):
            #         # find = max(person_count, key=lambda x: list(x.values()))
            #         # print(find)
            #         find = max(person_count, key=(lambda item: item['area']))
            #         xma = find['xmax']
            #         xmi = find['xmin']
            #         yma = find['ymax']
            #         ymi = find['ymin']
            #         cx = int((xma+xmi)/2)
            #         cy = int((ymi+yma)/2)    
            #         err = cx - w/2
            #         vel.linear.x = 0.16
            #         vel.angular.z = -float(err) / 400 
            #         node.cmdvel_publisher_.publish(vel)
            #         node.get_logger().info("Sending velocity: " + str(vel.angular.z))

                # print(person_count)



            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)
            object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
            label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

    cv2.putText(frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
    # cv2.imshow('Object detector', frame)

    t2 = cv2.getTickCount()
    time1 = (t2-t1)/freq
    frame_rate_calc= 1/time1
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
videostream.stop()
