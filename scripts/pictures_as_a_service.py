#!/usr/bin/env python3

from ultralytics import YOLO
from cv_bridge import CvBridge
from rebet_msgs.srv import DetectObject
from rebet_msgs.srv import TakePictures
import rclpy
import cv2
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rebet_msgs.msg import ObjectsIdentified
import time
import math
import random

DARKNESS_AMPLITUDE = 80 #half the max
DARKNESS_FREQUENCY = 0.006

class PicturesAsAService(Node):

    def __init__(self):
        super().__init__('take_pictures')
        self.start_time = time.time()
        self.darkness_gen = self.current_darkness()

        self.get_logger().info('created')

    def current_darkness(self):
        while True:
            time_since_start = time.time() - self.start_time

            yield DARKNESS_AMPLITUDE * math.sin(2 * math.pi * DARKNESS_FREQUENCY * time_since_start) + DARKNESS_AMPLITUDE

    def on_activate(self, state: State):
        self.srv = self.create_service(TakePictures, 'take_pictures_srv', self.take_pictures_service, callback_group = MutuallyExclusiveCallbackGroup())
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State):
        self.get_logger().info('on_deactivate() is called.')
        
        return TransitionCallbackReturn.SUCCESS if self.destroy_service(self.srv) else TransitionCallbackReturn.ERROR

    def take_pictures_service(self, request, response):
        rate = request.rate
        self.get_logger().info('Incoming request')
        curr_dark = next(self.darkness_gen)
        lighting = 1 - curr_dark/(DARKNESS_AMPLITUDE*2)
        good = 0
        for _ in range(rate):
            if random.random() < lighting:
                good += 1
        
        response.result = good

        self.get_logger().info('sending response')
        
        return response
        
def main():
    rclpy.init()

    minimal_service = PicturesAsAService()
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(minimal_service)
    mt_executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()