# Gaze Direction Prediction Model
This repository contains the project of human gaze prediction in robotics.
![image](https://github.com/user-attachments/assets/f2dd0ac5-3041-47dc-9492-7fe67268c1a2)

## Overview
This repository contains a project predicting a person's gaze direction, inspired by the research paper "Humans Have an Expectation That Gaze is Directed Toward Them" by Isabelle Mareschal, Andrew J. Calder, and Colin W.G. Clifford. The project applies an algorithmic approach based on conditional logic to model human perception of gaze direction, emphasizing how humans tend to assume others' gaze is directed towards them, particularly under conditions of uncertainty (e.g., low visibility, noisy images).

The project is implemented using ROS (Robot Operating System) to create a modular, scalable framework that allows real-time gaze prediction with various inputs. ROS 2 provides the communication infrastructure, and the gaze prediction algorithm is encapsulated in reusable nodes.

The model takes as inputs head orientation, eye orientation, and noise in eye detection (e.g., due to occlusion or noise, such as sunglasses), and predicts the perceived gaze direction as well as whether the gaze is classified as direct.

## Key Components
### 1. Model Overview

![image](https://github.com/user-attachments/assets/2ed54e6a-116c-47d4-820d-b6315feed4dd)


the gaze prediction model follows an algorithmic approach based on conditional logic to predict gaze direction and determine whether the gaze is direct or non-direct. The model takes into account three key inputs and produces two outputs, which are influenced by specific dependencies and conditions.

#### Inputs:
1. Head Orientation: The head's position is modeled as a sinusoidal wave with a range of ±30 degrees. This simulates head rotations.

2. Eye Orientation: Similarly, the eye orientation is represented as a sinusoidal wave with a range of ±12 degrees, simulating the movements of the eyes within the head.

3. Eye Noise: Eye noise is represented by a toggle, where:

* 0 represents low noise (clear eye input).
* 1 represents high noise (obscured or uncertain eye input, e.g., due to sunglasses).
#### Outputs:
1. Gaze Direction: The model calculates the direction of the gaze in degrees.

2. Direct Gaze: The model predicts whether the gaze is classified as direct (towards the observer) or non-direct (averted gaze).

### 2. Conditional Dependency:
1. Gaze Direction:

When eye noise is low (0), the model combines both head orientation and eye orientation to compute the gaze direction.
When eye noise is high (1), the model uses head orientation only to determine the gaze direction, as eye orientation becomes unreliable due to the noise.

2. Direct Gaze:

The model classifies the gaze as direct if the calculated gaze direction falls within a selected threshold of ±10 degrees. If the gaze direction is outside this threshold, the gaze is classified as non-direct.

## Acknowledgments
This project is inspired by the paper "Humans Have an Expectation That Gaze is Directed Toward Them" by Mareschal et al.
