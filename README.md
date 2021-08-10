# Ping-Pong-Robot-Computer-Vision
This code is related to the "Racket Sports with a Humanoid Robot" University of New Hampshire 2020 senior project.

# Overview
The goal of the computer vision portion of this project was to accurately locate a ping pong ball, and calculate it's distance with respect to our robot. An Intel® RealSense™ Depth Camera D435i for the streaming of images.

# Code Functionality
This code is the portion of the computer vision algorithm that calculates the location of a ping pong ball in pixel coordinates.
- using color based image segmentation, find candidate pixels which could represent the ping pong ball 
- using blob detection, compare groups of pixels that were selected via image segmentation and determine which ones represent the ping pong ball
- calculate the centroid of the blob which represents the ping pong ball and publish it's coordinates via ROS 

# Main Packages Used
- OpenCV
- ROS
