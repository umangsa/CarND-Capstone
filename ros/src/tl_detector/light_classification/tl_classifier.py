from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
import rospkg
import os
import numpy as np

def load_graph(frozen_graph_filename, prefix, input_map=None):
    # We load the protobuf file from the disk and parse it to retrieve the 
    # unserialized graph_def
    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    # Then, we import the graph_def into a new Graph and returns it 
    with tf.Graph().as_default() as graph:
        # The name var will prefix every op/nodes in your graph
        # Since we load everything in a new graph, this is not needed
        tf.import_graph_def(graph_def, input_map=input_map, name=prefix)
    return graph

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('tl_detector')
        self.class_threshold = 0.8

        self.config = tf.ConfigProto()
        # self.config.gpu_options.allow_growth = True
        # self.config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        self.detection_graph = load_graph(os.path.join(pkg_path, "light_classification", "detection_graph.pb"), "detection")
        
        self.detection_image_tensor = self.detection_graph.get_tensor_by_name('detection/image_tensor:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection/detection_scores:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection/detection_boxes:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection/detection_classes:0')

        self.run_tensors = [self.detection_boxes, self.detection_scores, self.detection_classes]


        self.detection_sess = tf.Session(graph=self.detection_graph, config=self.config)



        self.classification_graph = tf.Graph()
        self.classification_sess = tf.Session(graph=self.classification_graph, config=self.config)

        with self.classification_graph.as_default() as graph, self.classification_sess.as_default() as sess:
            self.boxes = tf.placeholder(tf.float32, (None, 4))
            self.classification_image = tf.placeholder(tf.float32, (None, None, None, 3))
            self.box_indices = tf.placeholder(tf.int32, (None))
            lights = tf.image.crop_and_resize(self.classification_image, self.boxes, self.box_indices, (32, 32))

            with tf.gfile.GFile(os.path.join(pkg_path, "light_classification", "classification_graph.pb"), "rb") as f:
                graph_def = tf.GraphDef()
                graph_def.ParseFromString(f.read())

            tf.import_graph_def(graph_def, input_map={"input_image:0": lights, "keep_prob:0": tf.constant(1.0), "phase_train:0": tf.constant(False)}, name="classification")

        self.classification_predictions = self.classification_graph.get_tensor_by_name('classification/predictions:0')

        self.im_count = 0
        black = np.zeros((600, 800, 3), dtype=np.uint8)
        _ = self.get_classification(black)
        print("classifier ready")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        # return TrafficLight.RED
        # cv2.imwrite("/udacity/temp/{}.jpg".format(self.im_count), image)
        self.im_count = self.im_count + 1
        # image = cv2.resize(image, (400, 300), cv2.INTER_AREA)
        image = image[0:400, 0:800]
        image = np.expand_dims(image, axis=0)
        with self.detection_sess.as_default(), self.detection_graph.as_default():
          feed_dict = {self.detection_image_tensor: image}
          boxes, scores, classes = self.detection_sess.run(self.run_tensors, feed_dict=feed_dict)

        boxes = boxes[0]
        scores = scores[0]
        classes = classes[0]

        boxes = boxes[(scores > self.class_threshold) & (classes == 10)]
        scores = scores[(scores > self.class_threshold) & (classes == 10)]
        # print("get_classification boxes = {}".format(boxes.shape[0]))

        if boxes.shape[0] > 0:
            return TrafficLight.RED
            # with self.classification_graph.as_default() as graph, self.classification_sess.as_default() as sess:
            #     predictions = sess.run(self.classification_predictions, feed_dict={self.boxes: boxes, 
            #                                                                        self.classification_image: im, 
            #                                                                        self.box_indices: np.zeros(boxes.shape[0], dtype=np.int32)})
            # print("predictions = {}".format(predictions))
            # final_class = TrafficLight.UNKNOWN
            # for pred in predictions:
            #     if pred == 0:
            #         final_class = TrafficLight.RED
            #         break
            #     elif pred == 1:
            #         final_class = TrafficLight.YELLOW
            #     elif pred == 2 and final_class != TrafficLight.YELLOW:
            #         final_class = TrafficLight.GREEN
            # return final_class
        else:
            return TrafficLight.RED
