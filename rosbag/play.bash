#!/bin/bash

ros2 bag play rosbag2_2024_07_11-16_34_43 -r 5 -l --remap /image_raw:=/theta/image_raw/compressed
