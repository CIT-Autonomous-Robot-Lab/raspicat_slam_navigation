#!/bin/bash
# install yq
# sudo wget https://github.com/mikefarah/yq/releases/download/v4.40.1/yq_linux_amd64 -O /usr/local/bin/yq
# sudo chmod +x /usr/local/bin/yq

yaml_file="$HOME/mugimaru-docker/ros2_ws/src/raspicat_slam_navigation/raspicat_navigation/config/waypoint/wp_tsukuba_kakuninn_goal.yaml" # 追加分のwaypointのyamlのパス

# Read YAML file and extract waypoints using yq tool (assuming it's installed)
waypoints=($(yq eval '.waypoints | keys | .[]' "$yaml_file"))

for waypoint in "${waypoints[@]}"; do
    pose_x=$(yq eval ".waypoints.$waypoint.pose[0]" "$yaml_file")
    pose_y=$(yq eval ".waypoints.$waypoint.pose[1]" "$yaml_file")
    pose_z=$(yq eval ".waypoints.$waypoint.pose[2]" "$yaml_file")
    orientation_w=$(yq eval ".waypoints.$waypoint.orientation[0]" "$yaml_file")
    orientation_x=$(yq eval ".waypoints.$waypoint.orientation[1]" "$yaml_file")
    orientation_y=$(yq eval ".waypoints.$waypoint.orientation[2]" "$yaml_file")
    orientation_z=$(yq eval ".waypoints.$waypoint.orientation[3]" "$yaml_file")

    # Send goal using ros2 action
#<< COMMENTOUT
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
    pose:
      header: 
        stamp: 
          sec: 0
        frame_id: 'map'
      pose:
        position: { x: $pose_x, y: $pose_y, z: $pose_z}
        orientation: { x: $orientation_x, y: $orientation_y, w: $orientation_w, z: $orientation_z}" &
#COMMENTOUT

    # debug
    echo "pose x: $pose_x, y: $pose_y, z: $pose_z"
    echo "orientation  x: $orientation_x, y: $orienation_y, w: $orientation_w, z: $orientation_z"

    # Wait for Enter key press to proceed to the next waypoint
    read -p "Press Enter to proceed to the next waypoint: "
done
