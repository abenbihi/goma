#!/bin/sh

scene_path=./data/sample_aachen/
image_path="$scene_path"/images/

output_path=./res/aachen/
line_segment_path="$output_path"line_segments/
vanishing_point_path="$output_path"vanishing_points/
config_path="$line_segment_path"/config.txt

echo "image_path: "$image_path""
echo "line_segment_path: "$line_segment_path""
echo "vanishing_point_path: "$vanishing_point_path""

mkdir -p "$vanishing_point_path"/db/
mkdir -p "$vanishing_point_path"/query/night/nexus5x/

./bin/goma vanishing_point_detector \
  --Reader.config_path "$config_path" \
  --Reader.scene_path "$scene_path" \
  --Reader.image_path "$image_path" \
  --Reader.output_path "$output_path" \
  --Reader.line_segment_path "$line_segment_path" \
  --Reader.vanishing_point_path "$vanishing_point_path" \
  --Reader.undistort 0 \
  --Reader.undistortion_max_size -1 \
  --VanishingPoint.min_num_inliers 10 \
  --VanishingPoint.min_remaining_lines_ratio 0.2 \
  --VanishingPoint.min_line_segment_size 50 \
  --VanishingPoint.min_line_segment_score 0.7 \
  --VanishingPoint.line_orientation_tolerance 0.2 \
  --VanishingPoint.max_vanishing_points_num 10 \
  --VanishingPoint.max_error 1 \
  --VanishingPoint.max_error2 2 \
  --VanishingPoint.confidence 0.999\
  --VanishingPoint.min_num_trials 10000 \
  --VanishingPoint.max_num_trials 100000 \
  --VanishingPoint.min_inlier_ratio 0.1
