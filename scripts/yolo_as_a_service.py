#!/usr/bin/env python3

from ultralytics import YOLO
from cv_bridge import CvBridge
from rebet_msgs.srv import DetectObject
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

DETECT_MODEL_PARAM = "detect_model_name"

class YoloAsAService(Node):

    def __init__(self):
        super().__init__('detect_object')


        self.bridge = CvBridge()

        self.declare_parameter(DETECT_MODEL_PARAM, "yolov8n")
        self.current_model_name = self.get_parameter(DETECT_MODEL_PARAM).get_parameter_value().string_value

        package_name = "rebet_frog"
        weight_dir_x = get_package_share_directory(package_name) + "/config/" + "yolov8x.pt" 
        weight_dir_n = get_package_share_directory(package_name) + "/config/" + "yolov8n.pt" 

        self.models = {}
        self.models["yolov8x"] = YOLO(weight_dir_x)
        self.models["yolov8n"] = YOLO(weight_dir_n)

        self.topic_name = "/camera/image_noisy"

        self.image_received = None
        self.get_logger().info('created')

    def create_image_subscriber(self):
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            10, callback_group = MutuallyExclusiveCallbackGroup())


    def parameter_changed_callback(self, params):
        for param in params:
            if(param.name == DETECT_MODEL_PARAM and param.value != self.current_model_name):
                self.current_model_name = param.value
                self.get_logger().info('Replaced current model used with new model: "%s"' % param.value)

        return SetParametersResult(successful=True)


    def on_activate(self, state: State):
        self.srv = self.create_service(DetectObject, 'detect_object_srv', self.detect_obj_service, callback_group = MutuallyExclusiveCallbackGroup())
        
        self.create_image_subscriber()
        self.add_on_set_parameters_callback(self.parameter_changed_callback)
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State):
        self.get_logger().info('on_deactivate() is called.')

        return TransitionCallbackReturn.SUCCESS if self.destroy_service(self.srv) else TransitionCallbackReturn.ERROR
    
    def listener_callback(self, msg):
        self.image_received = msg

    def detect_obj_service(self, request, response):
        self.get_logger().info('Incoming request')

        time.sleep(1)
        rgb_msg = self.image_received #assume to be new..

        obj_id = ObjectsIdentified()
        obj_id.object_detected = False
        if rgb_msg is not None:
            im = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            
            results = self.models[self.current_model_name].predict(source=im, save=False, save_txt=False, verbose=True)  # save predictions as labels
            obj_id.object_names = []
            obj_id.probabilities = []
            for i in range(len(results)):
                try:
                    obj_id.object_names.append(str(results[i].names[int(results[i].boxes.cls.cpu().numpy()[i])]))
                    obj_id.probabilities.append(float(results[i].boxes.conf.cpu().numpy()[i]))
                    self.get_logger().info(f'{str(results[i].names[int(results[i].boxes.cls.cpu().numpy()[i])])}  --  {float(results[i].boxes.conf.cpu().numpy()[i])}')
                except IndexError:
                    continue

            if(len(obj_id.object_names) > 0):
                obj_id.object_detected = True

        obj_id.stamp = self.get_clock().now().to_msg()
        obj_id.model_used = self.current_model_name
        response.objects_id = obj_id
        self.get_logger().info('sending response')
        
        response.id = request.id
        return response

def main():
    rclpy.init()

    minimal_service = YoloAsAService()
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(minimal_service)
    mt_executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()