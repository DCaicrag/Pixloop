#!/usr/bin/env python3

from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image

import cv2
import random
import numpy as np
import time

from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray

class CameraSubscriber:

    def __init__(self):
        rospy.init_node('camera_subscriber', anonymous=True)
        device = "cuda:0"  # cpu or cuda "cuda:0"
        self.threshold = 0.40
        self.enable = True
        self.device = device

        # Initialize FPS calculation variables
        self.last_time = time.time()
        self.fps = 0

        model = '/home/neousys/ros_drive/catkin_ws_tec/src/yolov8_py/scripts/yolov8n.pt'  # Update this path
        self.yolo = YOLO(model)
        self.yolo.fuse()
        self.yolo.to(device)
        # Verify and update this list to include all class names your model can detect
        self.yolo_class_names = ['person', 'bicycle', 'car', 'motorbike', 'aeroplane', 
                                 'bus', 'train', 'truck', 'boat', 'traffic light', 
                                 'fire hydrant', 'stop sign', 'parking meter', 'bench', 
                                 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 
                                 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 
                                 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 
                                 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 
                                 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 
                                 'bottle', 'wine glass', 'cup', 'fork', 'knife', 
                                 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 
                                 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 
                                 'donut', 'cake', 'chair', 'sofa', 'pottedplant', 
                                 'bed', 'diningtable', 'toilet', 'tvmonitor', 'laptop', 
                                 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 
                                 'oven', 'toaster', 'sink', 'refrigerator', 'book', 
                                 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 
                                 'toothbrush']  # Update this list based on your model

        # Create a mapping from class labels to integer IDs
        self.label_to_id = {label: idx for idx, label in enumerate(self.yolo_class_names)}


        # self.subscriber = rospy.Subscriber('/zed/zed_node/right/image_rect_color', Image, self.camera_callback)
        self.subscriber = rospy.Subscriber('/flir_adk/image_raw', Image, self.camera_callback)

        # self.subscriber = rospy.Subscriber('/zed/zed_node/right_raw/image_raw_color', Image, self.camera_callback)

        self.publisher = rospy.Publisher('/detections', Detection2DArray, queue_size=10)
        
        self._class_to_color = {}


    def camera_callback(self, data):
        # Check the encoding and adjust dtype and n_channels accordingly
        if data.encoding == 'bgra8':
            dtype = np.uint8
            n_channels = 4  # BGRA has 4 channels
        elif data.encoding == 'rgb8':
            dtype = np.uint8
            n_channels = 3
        # Add more elif blocks for other encodings if needed
        else:
            rospy.logwarn(f"Unsupported encoding: {data.encoding}")
            return

        # Construct a numpy array from the ROS Image message
        dtype = np.dtype(dtype)
        dtype = dtype.newbyteorder('>' if data.is_bigendian else '<')
        frame = np.ndarray(shape=(data.height, data.width, n_channels),
                        dtype=dtype, buffer=data.data)

        # Convert from BGRA to BGR if necessary
        if data.encoding == 'bgra8':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)



        img_copy = frame.copy()
        results = self.yolo.predict(
            source=img_copy,
            verbose=False,
            stream=False,
            conf=self.threshold,
            device=self.device
        )
        results: Results = results[0].cpu()

        # create detections msg
        detections_msg = Detection2DArray()
        detections_msg.header = data.header

        for b in results.boxes:
            label = self.yolo.names[int(b.cls)]
            score = float(b.conf)


            if score < self.threshold or label not in self.label_to_id:
                continue



            detection = Detection2D()
            box = b.xywh[0]

            detection.bbox.center.x = float(box[0])
            detection.bbox.center.y = float(box[1])
            detection.bbox.size_x = float(box[2])
            detection.bbox.size_y = float(box[3])

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = self.label_to_id[label]  # Use the mapping to get the integer ID
            hypothesis.score = score
            detection.results.append(hypothesis)

            if label not in self._class_to_color:
                self._class_to_color[label] = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            color = self._class_to_color[label]


            min_pt = (round(detection.bbox.center.x - detection.bbox.size_x / 2.0), # round(detection.bbox.center.position.x - detection.bbox.size_x / 2.0
                        round(detection.bbox.center.y - detection.bbox.size_y / 2.0))
            max_pt = (round(detection.bbox.center.x + detection.bbox.size_x / 2.0),
                        round(detection.bbox.center.y + detection.bbox.size_y / 2.0))
            cv2.rectangle(frame, min_pt, max_pt, color, 2)


            label_text = "{} ({:.3f})".format(label, score)
            pos = (min_pt[0] + 5, min_pt[1] + 25)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, label_text, pos, font, 1, color, 1, cv2.LINE_AA)

            detections_msg.detections.append(detection)


        self.publisher.publish(detections_msg)


        # Calculate FPS
        current_time = time.time()
        self.fps = 1.0 / (current_time - self.last_time)
        self.last_time = current_time

        # Draw FPS on the frame
        cv2.putText(frame, f"FPS: {self.fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # If the image has color information, convert it from RGB to BGR
        if n_channels == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        cv2.imshow('result', frame)
        cv2.waitKey(10)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    o = CameraSubscriber()
    o.run()