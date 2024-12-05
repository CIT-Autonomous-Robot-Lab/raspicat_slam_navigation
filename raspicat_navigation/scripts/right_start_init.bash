#!usr/bin/bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: 49.94795608520508
      y: 69.70091247558594
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.8338248508777241
      w: 0.5520290916779126
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

