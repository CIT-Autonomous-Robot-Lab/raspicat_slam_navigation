#!usr/bin/bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: 468.09026551651897
      y: 367.4213395876261
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.5174900650139701
      w: 0.8556892149675821
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

