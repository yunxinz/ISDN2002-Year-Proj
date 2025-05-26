ros2 run octomap_server octomap_server_node \
  --ros-args \
   # grid size = 5cm
   # decrease for more precision, 
   # increase for faster processing
  -p resolution:=0.05 \
   # ignore points above z = 2.0m   
  -p max_height:=2.0 \ 
  -p frame_id:=camera_init \
  -p sensor_model.max_range:=10.0 \
  -p publish_2d_projection:=true \
  -p height_map:=true \
  # points below z = 0.2m are considered ground
  # relative to the ground plane (z = 0) not Lidar position 
  -p ground_filter_distance:=0.2 \
  -p ground_filter_min_points:=10 \
  -r cloud_in:=/cloud_registered \
  -r projected_map:=/octomap_2d