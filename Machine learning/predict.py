#!/usr/bin/env python
import keras
import rospy
from cnn import CNN
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from skimage.transform import resize
import numpy as np

import tensorflow as tf

graph = tf.get_default_graph()




class Predict:

    def __init__(self):

        rospy.init_node('predict')

        self.graph = tf.get_default_graph()

        self.bridge = CvBridge()

        poids = 'model.p'

        self.clf = None

        self.subscrib = rospy.Subscriber('/usb_cam/image_raw', Image, self.vision_subscribe)

        Width = 150
        Height = 150
        Depth = 3

        total_class = 6

        self.clf = CNN(Width=Width, Height=Height, Depth=Depth, total_classes=total_class).cnn_model()

        self.clf.load_weights(poids)

        print('\nCompiling model...')

        self.clf.compile(loss="sparse_categorical_crossentropy", optimizer='sgd', metrics=["accuracy"])

        rospy.spin()

    def vision_subscribe(self, img):

        cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        cv_img = resize(cv_img, (150, 150, 3))

        self.prediction_dice(np.array([cv_img]))

    def prediction_dice(self, image):
        global graph
        with graph.as_default():
            prediction = self.clf.predict(image, batch_size=1)
            classe = np.argmax(prediction)
            accuracy = prediction[0][classe]

            print accuracy, classe


if __name__ =='__main__':
    Predict()

