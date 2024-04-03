#!/usr/bin/env python3
import rospy
from Gaze_detection.msg import GazeData, results  # Assuming the class name is Results
import numpy as np

def determine_action(eye_noise, eye_orientation, head_orientation):
    # Check for Eye Noise
    if eye_noise == 0:
        # If Eye Noise is clear, action is to combine, regardless of eye and head orientation
        return np.add(eye_orientation, head_orientation)
    
    elif eye_noise == 1:
        # If Eye Noise is noisy, action is based on head orientation
        return head_orientation
    else:
        raise ValueError('Invalid Eye Noise')

# Initialize a publisher for gaze direction results
results_publisher = None

def gaze_data_callback(data):
    """
    Callback function that processes data received from the 'gaze_data_topic'.
    """
    global results_publisher

    gaze_direction = determine_action(data.noise_level, data.eye_angle, data.head_orientation)

    # Determine if the gaze direction is within the direct gaze threshold
    direct_gaze = 1 if -10 <= gaze_direction <= 10 else 0

    # Log the action
    rospy.loginfo(f"Action: {gaze_direction}, Direct Gaze: {direct_gaze}")

    # Publish the gaze direction results
    results_msg = results()
    results_msg.predicted_gaze_direction = gaze_direction
    results_msg.direct_gaze = direct_gaze  # Set the direct gaze value based on the condition
    results_publisher.publish(results_msg)


def receive_gaze_data():
    """
    Initializes the node, subscribes to the gaze_data_topic, and creates a publisher for the gaze direction results.
    """
    global results_publisher

    rospy.init_node('gaze_data_listener', anonymous=True)
    
    # Subscribe to the gaze_data_topic. This calls the gaze_data_callback function every time data is published on the topic.
    rospy.Subscriber('gaze_data_topic', GazeData, gaze_data_callback)
    
    # Initialize the publisher for publishing gaze direction results
    results_publisher = rospy.Publisher('gaze_direction_results_topic', results, queue_size=10)

    # Keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        receive_gaze_data()
    except rospy.ROSInterruptException:
        pass
