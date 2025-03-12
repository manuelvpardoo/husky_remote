#! /usr/bin/env python3

import cv2 as cv
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Thread
from math import pi
from rclpy.callback_groups import ReentrantCallbackGroup
# from pymoveit2 import MoveIt2
# from pymoveit2.robots import ur

class HandTeleopNode(Node):
    def __init__(self):
        super().__init__('hand_teleop_node')
        # add '/a200_0000/cmd_vel' to the publiser if you want to connect to the robot
        # add 'cmd_vel' to the publisher if you want to connect to the simulator
        self.publisher = self.create_publisher(Twist, '/a200_0000/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update)
        self.vid = cv.VideoCapture(0)
        self.mp_hands = mp.solutions.hands.Hands(model_complexity=0,
                                                 max_num_hands=1,
                                                 min_detection_confidence=0.5,
                                                 min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # # Initialize MoveIt2 for UR robot control
        # self.callback_group = ReentrantCallbackGroup()
        # self.moveit2 = MoveIt2(
        #     node=self,
        #     joint_names=["arm_0_shoulder_pan_joint", "arm_0_shoulder_lift_joint", "arm_0_elbow_joint", 
        #                  "arm_0_wrist_1_joint", "arm_0_wrist_2_joint", "arm_0_wrist_3_joint"],
        #     base_link_name="ur_base_link",
        #     end_effector_name="tool0",
        #     group_name="arm_0",
        #     callback_group=self.callback_group,
        # )
        # self.moveit2.max_velocity = 0.2
        # self.moveit2.max_acceleration = 0.1
        # self.movement_thread = None  # Thread for robot movement
        # self.movement_needed = False  # Flag to check if movement is requested
    
    def get_hand_move(self, hand_landmarks):
        landmarks = hand_landmarks.landmark
        # Check for closed fist: all fingertips are below their respective PIP joints
        is_fist_closed = all(
            landmarks[tip_id].y > landmarks[tip_id - 2].y
            for tip_id in [8, 12, 16, 20]  # Fingertip indices
        )
        if is_fist_closed:
            return "backward"
    
        if all(landmarks[i].y > landmarks[i + 3].y for i in range(5, 20, 4)):
            return "forward"
        elif landmarks[5].y > landmarks[9].y and landmarks[9].y > landmarks[13].y and landmarks[13].y > landmarks[17].y:
            return "turn left"
        elif landmarks[5].y < landmarks[9].y and landmarks[9].y < landmarks[13].y and landmarks[13].y < landmarks[17].y:
            return "turn right"
        # elif landmarks[13].y < landmarks[16].y and landmarks[17].y < landmarks[20].y and landmarks[5].y > landmarks[8].y and landmarks[9].y > landmarks[12].y:
        #     return "home position"
        else:
            return "unknown"

    # def move_ur_home(self):
    #     home_position = [0.0, -pi / 2, pi / 2, 0.0, 0.0, 0.0]
    #     self.get_logger().info("Moving UR robot to home position")
    #     traj = self.moveit2.move_to_configuration(home_position)
    #     if traj is None:
    #         self.get_logger().error("Failed to move to home position")
    #     else:
    #         self.moveit2.execute(traj)
    #         success = self.moveit2.wait_until_executed()
    #         if not success:
    #             self.get_logger().error("Failed to execute trajectory")
    #         else:
    #             self.get_logger().info("UR robot moved to home position")
    
    def update(self):
        ret, frame = self.vid.read()
        if not ret or frame is None:
            return

        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.mp_hands.process(frame)
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        black_frame = np.zeros_like(frame)
        hand_move = "no hand detected"

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(black_frame, 
                                               hand_landmarks, 
                                               mp.solutions.hands.HAND_CONNECTIONS,
                                               self.mp_drawing_styles.get_default_hand_landmarks_style(),
                                               self.mp_drawing_styles.get_default_hand_connections_style())
                hand_move = self.get_hand_move(hand_landmarks)

        black_frame = cv.flip(black_frame, 1)
        cv.putText(black_frame, "Hand Move: " + hand_move, (50, 50), 
                   cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2, cv.LINE_AA)
        cv.imshow('Hand Teleop', black_frame)

        twist = Twist()
        if hand_move == "forward":
            twist.linear.x = 0.1

        elif hand_move == "backward":
            twist.linear.x = -0.1  # Negative for backward movement

        elif hand_move == "turn left":
            twist.angular.z = 0.1
        elif hand_move == "turn right":
            twist.angular.z = -0.1
        # elif hand_move == "home position" and not self.movement_needed:
        #     self.movement_needed = True
        #     # Run the UR movement in a separate thread
        #     self.movement_thread = Thread(target=self.move_ur_home)
        #     self.movement_thread.start()
        else:
            twist.linear.x = 0.0

        self.publisher.publish(twist)

        if cv.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            self.vid.release()
            cv.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HandTeleopNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
