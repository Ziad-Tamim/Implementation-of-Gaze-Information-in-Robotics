#!/usr/bin/env python3
import rospy
from Gaze_detection.msg import GazeData
import numpy as np

def determine_action(eye_noise, eye_orientation, head_orientation):
    # Check for Eye Noise
    if eye_noise == 0:
        # If Eye Noise is clear, action is to combine, regardless of eye and head orientation
        return "combined(Head and eye)", np.add(eye_orientation, head_orientation)
    
    elif eye_noise == 1:
        # If Eye Noise is noisy, action is based on head orientation
        return "head_orientation", head_orientation
    else:
        raise ValueError('Invalid Eye Noise')

def gaze_data_callback(data):
    """
    Callback function that processes data received from the 'gaze_data_topic'.
    """
    # rospy.loginfo(f"Received gaze data: Eye Angle={data.eye_angle}, Head Orientation={data.head_orientation}, Noise Level={data.noise_level}")

    gaze_direction = determine_action(data.noise_level, data.eye_angle, data.head_orientation)

    rospy.loginfo(f"Action: {gaze_direction}")


def receive_gaze_data():
    """
    Initializes the node, subscribes to the gaze_data_topic, and spins to keep the script from exiting.
    """
    rospy.init_node('gaze_data_listener', anonymous=True)
    
    # Subscribe to the gaze_data_topic. This calls the gaze_data_callback function every time data is published on the topic.
    rospy.Subscriber('gaze_data_topic', GazeData, gaze_data_callback)
    
    # Keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        receive_gaze_data()
    except rospy.ROSInterruptException:
        pass
