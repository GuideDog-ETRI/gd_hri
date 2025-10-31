#!/bin/bash

# -r 2 # 2배속 재생
# -l    # 루프 반복 재생

# ros2 bag play /mnt/d/20250523_rosbag/rosbag2_2025_05_23-14_25_42-신호등찾아되는곳 -r 2 -l

## no theta, only have /zed/zed_node/left/image_rect_color/compressed
# ros2 bag play /mnt/d/240823_자체데이터셋획득_1-노은역/rosbag2_2024_08_23-14_33_22 -r 2 -l --remap /zed/zed_node/left/image_rect_color/compressed:=/theta/image_raw/compressed
ros2 bag play /mnt/d/240823_자체데이터셋획득_1-노은역/rosbag2_2024_08_23-14_33_22 -r 30 -l

## no theta, only have /zed/zed_node/left_raw/image_raw_color/compressed
# ros2 bag play /mnt/d/240729_jackal_GDM/rosbag2_2024_07_29-13_01_05 -r 2 -l

# ros2 bag play /mnt/d/20250707_원내실외_scout_ROSBAGS/my_camera_bag_20250707_104701 -r 2 -l --topics /theta/image_raw/compressed /camera/camera/color/image_raw