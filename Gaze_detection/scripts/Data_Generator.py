#!/usr/bin/env python3
import rospy
from Gaze_detection.msg import GazeData
import numpy as np


def generate_data():
    rospy.init_node('gaze_data_generator', anonymous=True)
    pub = rospy.Publisher('gaze_data_topic', GazeData, queue_size=10)
    
    # Define the sinusoid frequencies in Hz
    eye_frequency = 0.45  # Eye angle signal frequency
    head_frequency = 0.1  # Head orientation signal frequency
    
    # Define the time step for publishing the messages
    rate = rospy.Rate(10)  # Publishing rate in Hz
    
    eye_angle_phase = 0
    head_orientation_phase = 0
    
    # Define how much the phase increases on each iteration
    eye_angle_phase_increment = 2 * np.pi * eye_frequency / 10
    head_orientation_phase_increment = 2 * np.pi * head_frequency / 10

    toggle_counter = 0  # Initialize a counter for toggling the noise level
    
    while not rospy.is_shutdown():
        # Update the phases
        eye_angle_phase += eye_angle_phase_increment
        head_orientation_phase += head_orientation_phase_increment
        
        # Reset the phase after completing a full cycle to prevent overflow
        eye_angle_phase = eye_angle_phase % (2 * np.pi)
        head_orientation_phase = head_orientation_phase % (2 * np.pi)
        
        # Generate the sinusoidal data
        eye_angle = 12 * np.sin(eye_angle_phase)
        head_orientation = 30 * np.sin(head_orientation_phase)
        
        # Generate the noise level
        # Toggle the noise level every 10 iterations (1 Hz)
        if toggle_counter % 30 == 0:
            noise_level = 1 if toggle_counter // 30 % 2 == 0 else 0
    
        # Create the GazeData message
        gaze_data = GazeData()
        gaze_data.eye_angle = eye_angle
        gaze_data.head_orientation = head_orientation
        gaze_data.noise_level = noise_level
        
        # Publish the message
        pub.publish(gaze_data)
        
        # Sleep to maintain the publishing rate
        rate.sleep()

        # Increment the counter
        toggle_counter += 1

if __name__ == '__main__':
    try:
        generate_data()
    except rospy.ROSInterruptException:
        pass
