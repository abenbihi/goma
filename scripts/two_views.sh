#!/bin/sh

scene_path=./data/sample_aachen/
image_path="$scene_path"/images/

output_path=./data/sample_aachen/
line_segment_path="$output_path"line_segments/
vanishing_point_path="$output_path"vanishing_points/
plane_path="$output_path"planes/
feature_path="$output_path"/features/
match_path="$output_path"/point_matches/

config_path="$line_segment_path"/config.txt

echo "image_path: "$image_path""
echo "line_segment_path: "$line_segment_path""
echo "vanishing_point_path: "$vanishing_point_path""
echo "plane_path: "$plane_path""

./bin/goma planewise_homography_estimator \
  --Reader.scene_path "$scene_path" \
  --Reader.image_path "$image_path" \
  --Reader.output_path "$output_path" \
  --Reader.undistort 0 \
  --Reader.image_id1 2723 \
  --Reader.image_id2 5307 \
  --Reader.undistortion_max_size -1 \
  --Reader.line_segment_path "$line_segment_path" \
  --Reader.vanishing_point_path "$vanishing_point_path" \
  --Reader.plane_path "$plane_path" \
  --Reader.feature_path "$feature_path" \
  --Reader.match_path "$match_path" \
  --Reader.config_path "$config_path" \
  --FeatureMatching.min_num_trials 100 \
  --FeatureMatching.confidence 0.999 \
  --FeatureMatching.max_num_trials 10000 \
  --FeatureMatching.min_inlier_ratio 0.1
