#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from yolo_ros.msg import BoundingBox, BoundingBoxes

import torch
import cv2
import numpy as np

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_inference_node')
        rospy.loginfo("Nodo YOLO iniciado...")

        # === Parámetros ROS ===
        self.weight_path = rospy.get_param('~weights_path')
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.3)
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.bounding_boxes_topic = rospy.get_param('~bounding_boxes_topic', '/yolo/bounding_boxes')
        self.detection_image_topic = rospy.get_param('~detection_image_topic', '/yolo/detection_image')
        self.publish_annotated_image = rospy.get_param('~publish_annotated_image', True)

        rospy.loginfo(f"Pesos: {self.weight_path}")
        rospy.loginfo(f"Confianza mínima: {self.conf_threshold}")

        # === Carga del modelo YOLOv5 desde Torch Hub ===
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.weight_path, force_reload=False)
        self.model.conf = self.conf_threshold
        self.class_names = self.model.names

        # === Comunicación ROS ===
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.bbox_pub = rospy.Publisher(self.bounding_boxes_topic, BoundingBoxes, queue_size=10)

        if self.publish_annotated_image:
            self.annotated_img_pub = rospy.Publisher(self.detection_image_topic, Image, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Error en conversión de imagen: {e}")
            return
        
        results = self.model(cv_image)
        detections = results.xyxy[0].cpu().numpy()

        bbox_msg = BoundingBoxes()
        bbox_msg.header = msg.header

        for det in detections:
            xmin, ymin, xmax, ymax, conf, cls_id = det

            if conf < self.conf_threshold:
                continue

            box = BoundingBox()
            box.class_name = self.class_names[int(cls_id)]
            box.probability = float(conf)
            box.xmin = float(xmin)
            box.ymin = float(ymin)
            box.xmax = float(xmax)
            box.ymax = float(ymax)

            bbox_msg.bounding_boxes.append(box)

            if self.publish_annotated_image:
                cv2.rectangle(cv_image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                label = f'{box.class_name} {box.probability:.2f}'
                cv2.putText(cv_image, label, (int(xmin), int(ymin) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.bbox_pub.publish(bbox_msg)

        if self.publish_annotated_image:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                annotated_msg.header = msg.header
                self.annotated_img_pub.publish(annotated_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Error publicando imagen: {e}")

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
