ros2 launch marathon_ros2_bringup nav2_tiago_launch.py map:=map.yaml params_file:=nav2_conf.yaml use_rviz:=false

ros2 run person_detector person_detector_test --ros-args -p feature_weights_path:=triplet_weights.pth -p yolo_weights_path:=yolov3.weights

