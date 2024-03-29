"""
Object Detection Module using Ultralytics YOLO

This module uses the Ultralytics implementation of YOLO for object detection.
It's designed for use in applications requiring real-time object detection.

Classes:
    ObjectDetection: Manages YOLO model for object detection.
"""
import os
from ultralytics import YOLO
import torch
import yaml
import random
from random import randint


class ObjectDetection:
    """
    Handles YOLO model loading with specified weights, performs object
    detection on images, and processes detection results.

    Attributes:
        config (dict): Configuration parameters.
        device (str): Computation device ('cuda' or 'cpu').
        weight_path (str): Path to model weights file.
        conf_thresh (float): Confidence threshold for detections.
        iou_thresh (float): IOU threshold for non-maximum suppression.
        model: Loaded YOLO model for inference.
    """
    def __init__(self, config_path):
        """
        Initializes ObjectDetection with configurations from YAML file.

        Args:
            config_path (str): Path to configuration YAML.
        """
        self.config = self.load_config(config_path)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # Adjust the weights path to be absolute
        self.weight_path = self.config['object_detection']['weight_path']
        self.conf_thresh = self.config['object_detection']['conf_thresh']
        self.iou_thresh = self.config['object_detection']['iou_thresh']
        self.model = self.load_model()

    def load_config(self, config_path):
        """
        Loads configuration from YAML.

        Args:
            config_path (str): Configuration file path.

        Returns:
            dict: Loaded configuration.
        """
        try:
            with open(config_path, 'r', encoding='utf-8') as file:
                return yaml.safe_load(file)
        except FileNotFoundError as exc:
            error_msg = f"The configuration file {config_path} was not found."
            raise FileNotFoundError(error_msg) from exc
        except PermissionError as exc:
            error_msg = f"Permission denied when accessing {config_path}."
            raise PermissionError(error_msg) from exc
        except yaml.YAMLError as exc:
            error_msg = "Error parsing YAML configuration."
            raise RuntimeError(f"{error_msg}: {exc}") from exc

    def load_model(self):
        """
        Loads and fuses YOLO model.

        Returns:
            Loaded YOLO model.
        """
        if not os.path.exists(self.weight_path):
            error_msg = f"Model weights file {self.weight_path} not found."
            raise FileNotFoundError(error_msg)

        try:
            model = YOLO(self.weight_path)
            model.fuse()
            return model
        except Exception as exc:
            error_msg = "Failed to load model with error."
            raise RuntimeError(f"{error_msg}: {exc}") from exc

    def predict(self, frame):
        """
        Performs detection on image frame.

        Args:
            frame: Image frame for detection.

        Returns:
            Detection results.
        """
        results = self.model.predict(frame, show=False, conf=self.conf_thresh,
                                     iou=self.iou_thresh, device=self.device)
        return results

    def extract_detections(self, results):
        """
        Extracts bounding boxes, class names, and confidence scores.

        Args:
            results: Detection results.

        Returns:
            List of tuples with class name, confidence, bbox, and class ID.
        """
        # pylint: disable=unsubscriptable-object
        return [(self.model.names[int(box.cls[0])],
                box.conf[0],
                tuple(box.xywh[0]),
                i)
                for result in results
                for i, box in enumerate(result.boxes.cpu().numpy())]

    def generate_color(self):
        """
        Generates unique random RGB colors for the specified number of classes.

        Returns:
            list: A list of unique RGB color tuples for each class.
        """
        random.seed(42)
        colors = set()

        while len(colors) < len(self.model.names):
            # Generate a new random color
            new_color = (randint(0, 255), randint(0, 255), randint(0, 255))
            colors.add(new_color)
        return list(colors)
