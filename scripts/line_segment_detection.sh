#!/bin/sh

scene_path=./data/sample_aachen/
image_path="$scene_path"/images/

output_path=./res/aachen/
line_segment_path="$output_path"line_segments/
config_path="$line_segment_path"/config.txt

echo "image_path: "$image_path""
echo "line_segment_path: "$line_segment_path""

mkdir -p "$line_segment_path"/db/
mkdir -p "$line_segment_path"/query/night/nexus5x/

./bin/goma line_segment_detector \
  --Reader.scene_path "$scene_path" \
  --Reader.image_path "$image_path" \
  --Reader.output_path "$output_path" \
  --Reader.undistort 0 \
  --Reader.undistortion_max_size -1 \
  --Reader.exec_mode 1 \
  --Reader.line_segment_path "$line_segment_path" \
  --Reader.config_path "$config_path" \
  --VanishingPoint.min_line_segment_size 20 \
  --VanishingPoint.min_line_segment_score 0.7
