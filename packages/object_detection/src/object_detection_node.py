#!/usr/bin/env python3

import cv2
import numpy as np
from typing import Tuple
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, EpisodeStart
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

from nn_model.constants import IMAGE_SIZE
from nn_model.model import Wrapper

from integration_activity import \
    NUMBER_FRAMES_SKIPPED, \
    filter_by_classes, \
    filter_by_bboxes, \
    filter_by_scores


class ObjectDetectionNode(DTROS):
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ObjectDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.initialized = False
        self.log("Initializing!")

        self.veh = rospy.get_namespace().strip("/")
        #self.avoid_duckies = False
        self.detected_class = 1000000

        # Construct publishers
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic,
            Twist2DStamped,
            queue_size=1,
            dt_topic_type=TopicType.CONTROL
        )

        # episode_start_topic = f"/{self.veh}/episode_start"
        # rospy.Subscriber(
        #     episode_start_topic,
        #     EpisodeStart,
        #     self.cb_episode_start,
        #     queue_size=1
        # )

        self.pub_detections_image = rospy.Publisher(
            "~image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        # Construct subscribers
        self.sub_image = rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1,
        )

        self.bridge = CvBridge()
        self.initial_speed = 0.4
        self.v = rospy.get_param("~speed", 0.0)
        aido_eval = rospy.get_param("~AIDO_eval", False)
        self.log(f"AIDO EVAL VAR: {aido_eval}")
        self.log("Starting model loading!")
        self._debug = rospy.get_param("~debug", False)
        self.model_wrapper = Wrapper(aido_eval)
        self.log("Finished model loading!")
        self.time_begin_stop = 0
        self.time_begin_speed_lim = 0
        self.stopped = False
        self.following_speed_lim = False
        #self.duration = (rospy.Duration(4)).to_sec()
        self.frame_id = 0
        self.first_image_received = False
        self.initialized = True
        self.log("Initialized!")

        rospy.on_shutdown(self._on_shutdown)

    def image_cb(self, image_msg):
        if not self.initialized:
            #self.pub_car_commands(True, image_msg.header)
            car_control_msg = Twist2DStamped()  
            car_control_msg.v = 0
            car_control_msg.omega = 0
            self.pub_car_cmd.publish(car_control_msg)
            return
        
        names = ['Speed Limit 20 KMPh', 'Speed Limit 30 KMPh', '50 mph speed limit', 'Stop_Sign', 'Turn right ahead', 'Turn left ahead', 'Duckie']

        self.frame_id += 1
        self.frame_id = self.frame_id % (1 + NUMBER_FRAMES_SKIPPED())
        #self.frame_id = self.frame_id % (1 + 1)
        if self.frame_id != 0:
            #self.pub_car_commands(self.avoid_duckies, image_msg.header)
            self.pub_car_commands(self.detected_class, image_msg.header)
            return

        # Decode from compressed image with OpenCV
        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr("Could not decode image: %s" % e)
            return

        rgb = bgr[..., ::-1]

        rgb = cv2.resize(rgb, (IMAGE_SIZE, IMAGE_SIZE))
        bboxes, classes, scores = self.model_wrapper.predict(rgb)

        detection = self.det2bool(bboxes, classes, scores)

        if detection and ((classes[0] == 0) or (classes[0] == 1) or (classes[0] == 2)):
            self.log("20 km/h speed limit sign detected .. slowing down to 20 km/h")
            #self.avoid_duckies = True
            print(classes[0])
            self.detected_class = 0

        if detection and (classes[0] == 3 or classes[0] == 6):
            self.log("Stop sign/ duckie detected .. stopping for 4 seconds")
            print(classes[0])
            self.detected_class = 3

        if detection and (classes[0] == 4):
            self.log("Turn right sign detected .. turning right")
            print(classes[0])
            self.detected_class = 4

        if detection and (classes[0] == 5):
            self.log("Turn left sign detected .. turning left")
            print(classes[0])
            self.detected_class = 5

        #self.pub_car_commands(self.avoid_duckies, image_msg.header)
        self.pub_car_commands(self.detected_class, image_msg.header)

        if self._debug:
            names = dict(enumerate(names))
            font = cv2.FONT_HERSHEY_SIMPLEX
            for clas, box in zip(classes, bboxes):
                pt1 = np.array([int(box[0]), int(box[1])])
                pt2 = np.array([int(box[2]), int(box[3])])
                pt1 = tuple(pt1)
                pt2 = tuple(pt2)
                #color = tuple(reversed(colors[clas]))
                color = 0, 255, 255
                name = names[clas]
                # draw bounding box
                rgb = cv2.rectangle(rgb, pt1, pt2, color, 2)
                # label location
                text_location = (pt1[0], min(pt2[1] + 30, IMAGE_SIZE))
                # draw label underneath the bounding box
                rgb = cv2.putText(rgb, name, text_location, font, 1, color, thickness=2)

            bgr = rgb[..., ::-1]
            obj_det_img = self.bridge.cv2_to_compressed_imgmsg(bgr)
            self.pub_detections_image.publish(obj_det_img)

    def det2bool(self, bboxes, classes, scores):
        box_ids = np.array(list(map(filter_by_bboxes, bboxes))).nonzero()[0]
        cla_ids = np.array(list(map(filter_by_classes, classes))).nonzero()[0]
        sco_ids = np.array(list(map(filter_by_scores, scores))).nonzero()[0]

        box_cla_ids = set(list(box_ids)).intersection(set(list(cla_ids)))
        box_cla_sco_ids = set(list(sco_ids)).intersection(set(list(box_cla_ids)))

        if len(box_cla_sco_ids) > 0:
            return True
        else:
            return False

    # def pub_car_commands(self, stop, header):
    #     car_control_msg = Twist2DStamped()
    #     car_control_msg.header = header
    #     #####put here the turn left,right, etc...
    #     if stop:
    #         car_control_msg.v = 0.0
    #     else:
    #         car_control_msg.v = self.v

    #     # always drive straight
    #     car_control_msg.omega = 0.0

    #     self.pub_car_cmd.publish(car_control_msg)

    def pub_car_commands(self, detected_class, header):
        car_control_msg = Twist2DStamped()
        car_control_msg.header = header
        #####put here the turn left,right, etc...
        if detected_class == 3: #stop, has highest priority
            car_control_msg.v = 0.0
            car_control_msg.omega = 0.0
            self.stopped = True
            self.time_begin_stop =  rospy.get_time()
        elif detected_class == 0: # speed lim
            #car_control_msg.v = self.v
            car_control_msg.v = 0.3
            car_control_msg.omega = 0.0
            #self.following_speed_lim = True
            self.time_begin_speed_lim =  rospy.get_time() 
        elif detected_class == 4: # right
            car_control_msg.v = 0.4 #self.initial_speed
            car_control_msg.omega = -12
        elif detected_class == 5: #left
            car_control_msg.v = 0.6 #self.initial_speed
            car_control_msg.omega = 40
        else:
            car_control_msg.v = self.initial_speed
            car_control_msg.omega = 0.0

        self.pub_car_cmd.publish(car_control_msg)
             

    def _on_shutdown(self):
        self.log("Received shutdown request.")
        self.is_shutdown = True
        # call node on_shutdown
        self.on_shutdown()

    def on_shutdown(self):
        # this function does not do anything, it is called when the node shuts down.
        # It can be redefined by the user in the final node class.
        #pass
        car_control_msg = Twist2DStamped()
        #car_control_msg.header = header
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.pub_car_cmd.publish(car_control_msg)


if __name__ == "__main__":
    # Initialize the node
    #os.system("pip install numpy --upgrade")
    object_detection_node = ObjectDetectionNode(node_name="object_detection_node")
    #rate = rospy.Rate(1)
    # Keep it spinning
    rospy.spin()
    #rate.sleep()
    #on_shutdown
