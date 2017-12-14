from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os
import numpy as np
import rospy
from functools import partial
import cv2
from random import randint

DIR_PATH = os.path.dirname(os.path.realpath(__file__))

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # load the graph
        
        PATH_TO_CKPT = DIR_PATH + '/frozen_inference_graph.pb'

        # rebuild chunks (chunk creating script from team vulture)
        if not os.path.exists(PATH_TO_CKPT):
            output = open(PATH_TO_CKPT, 'wb')
            chunks = os.listdir(DIR_PATH+'/frozen_model_chunks')
            chunks.sort()
            for filename in chunks:
                filepath = os.path.join(DIR_PATH+'/frozen_model_chunks', filename)
                with open(filepath, 'rb') as fileobj:
                    for chunk in iter(partial(fileobj.read, 1024), ''):
                        output.write(chunk)
            output.close()

        # current light default is unknown
        self.light_state = TrafficLight.UNKNOWN

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        # self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph=self.detection_graph)

    def get_classification(self, image):
        
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        # save image for testing model
        # rand_img_int = randint(0,100)
        # rospy.loginfo('save img');
        # rospy.loginfo(rand_img_int);
        # cv2.imwrite(DIR_PATH + '/data/camera_image' + str(state) + "-" + str(rand_img_int) + '.jpg', image)

        # prediction key
        classification_tl_key = {1: TrafficLight.RED, 2: TrafficLight.YELLOW, 3: TrafficLight.GREEN, 4: TrafficLight.UNKNOWN}

        # expand image shape for model
        image_np_expanded = np.expand_dims(image, axis=0)

        # run the prediction
        with self.detection_graph.as_default():

            (scores, classes) = self.sess.run(
            [self.detection_scores, self.detection_classes],
            feed_dict={self.image_tensor: image_np_expanded})

            # get the class with the highest score
            sorted_scores_classes = sorted(zip(scores[0], classes[0]), reverse=True)
            # if highest score is over the score threshold, assign that class as the light state
            if sorted_scores_classes[0][0] > 0.5:
                self.light_state = classification_tl_key[sorted_scores_classes[0][1]]
            else:
                self.light_state = TrafficLight.UNKNOWN
        


        return self.light_state