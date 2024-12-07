#!/bin/bash
# install yq
# sudo wget https://github.com/mikefarah/yq/releases/download/v4.40.1/yq_linux_amd64 -O /usr/local/bin/yq
# sudo chmod +x /usr/local/bin/yq

# 配列に複数のYAMLファイルを追加
yaml_files=(
  "$HOME/waypoint/wp_1.yaml"
  "$HOME/waypoint/wp_2.yaml"
  "$HOME/waypoint/wp_3.yaml"
  "$HOME/waypoint/wp_4.yaml"
  "$HOME/waypoint/wp_5.yaml"
  "$HOME/waypoint/wp_6.yaml"
  "$HOME/waypoint/wp_7.yaml"
  "$HOME/waypoint/wp_8.yaml"
  "$HOME/waypoint/wp_9.yaml"
  "$HOME/waypoint/wp_10.yaml"
  "$HOME/waypoint/wp_11.yaml"
  "$HOME/waypoint/wp_12.yaml"
  "$HOME/waypoint/wp_13.yaml"
  "$HOME/waypoint/wp_14.yaml"
)

# 各YAMLファイルを処理
for yaml_file in "${yaml_files[@]}"; do
    echo "Processing waypoints from: $yaml_file"
    # Read YAML file and extract waypoints using yq tool (assuming it's installed)
    waypoints=($(yq eval '.waypoints | keys | .[]' "$yaml_file"))

    # Waypointsを処理
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

        # デバッグ用
        echo "pose x: $pose_x, y: $pose_y, z: $pose_z"
        echo "orientation  x: $orientation_x, y: $orientation_y, w: $orientation_w, z: $orientation_z"
        
        # Enterキーで次のウェイポイントへ進む
        # read -p "Press Enter to proceed to the next waypoint: "
        wait
    done

    # YAMLファイル切り替え時のキー入力待ち
    echo "Finished processing $yaml_file."
    read -p "Press Enter to proceed to the next YAML file, or Ctrl+C to exit: "
    # read -p "Press 'n' and Enter to proceed to the next YAML file, or any other key to exit: " key
    # if [[ "$key" != "n" ]]; then
    #     echo "Exiting..."
    #     break
    # fi
done
