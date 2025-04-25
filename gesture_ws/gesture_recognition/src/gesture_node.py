#!/usr/bin/env python
import rospy
import cv2
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class GestureRecognition:
    def __init__(self):
        self.node_name = "gesture_recognition"
        rospy.init_node(self.node_name)

        # Publishers
        self.gesture_pub = rospy.Publisher('~gesture', String, queue_size=10)

        # Subscribers
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.mp_draw = mp.solutions.drawing_utils
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Convert the image to RGB
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(cv_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                gesture = self.recognize_gesture(hand_landmarks)
                rospy.loginfo(f"Gesture Recognized: {gesture}")
                self.gesture_pub.publish(gesture)

        cv2.imshow('MediaPipe Hands', cv_image)
        cv2.waitKey(1)

    def recognize_gesture(self, landmarks):
        # Implement gesture recognition logic here
        # Placeholder for demonstration
        return "Some Gesture"

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    gr = GestureRecognition()
    gr.run()
