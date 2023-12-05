// Copyright (c) 2023, Assia Benbihi
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of the <organization> nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author: Assia Benbihi (abenbihi-at-georgiatech-hyphen-metz-dot-fr)
//                       (assia-dot-benbihi-at-cvut-dot-cz)

#include "goma/estimators/vanishing_point.h"
#include "goma/optim/loransac.h"
#include "goma/scene/line_segment.h"

namespace goma {

void PreprocessLineSegments(
const VanishingPointOptions& vanishing_point_options,
LineSegments& line_segments){
  // Remove line segments that are too small.
  FilterLineSegments(
      vanishing_point_options.min_line_segment_score,
      vanishing_point_options.min_line_segment_size,
      line_segments);

  // Estimate line segment orientations.
  ClassifyLineSegmentOrientations(
      vanishing_point_options.line_orientation_tolerance,
      line_segments);
}

// Estimate one vanishing point.
VanishingPoint EstimateVanishingPoint(
    const VanishingPointOptions& options,
    const std::vector<LineSegment>& line_segments){
  // Format line segments.
  std::vector<Eigen::Vector3d> lines;
  for (size_t i = 0; i < line_segments.size(); ++i) {
    const auto line_segment = line_segments[i];
    const Eigen::Vector3d line_segment_start = line_segment.start.homogeneous();
    const Eigen::Vector3d line_segment_end = line_segment.end.homogeneous();
    const Eigen::Vector3d line = line_segment_start.cross(line_segment_end);
    lines.push_back(line);
  }

  // Estimate vanishing point.
  colmap::RANSACOptions ransac_options;
  ransac_options.max_error = options.max_error;
  ransac_options.confidence = options.confidence;
  ransac_options.min_num_trials = options.min_num_trials;
  ransac_options.max_num_trials = options.max_num_trials;
  ransac_options.min_inlier_ratio = options.min_inlier_ratio;
  goma::LORANSAC<VanishingPointEstimator,VanishingPointLocalEstimator>
    ransac(ransac_options);
  ransac.SetLocalError(options.max_error2);
  const auto report = ransac.Estimate(line_segments, lines);

  VanishingPoint vanishing_point;
  vanishing_point.id = -1; 
  if (!report.success){
    return vanishing_point;
  }

  vanishing_point.hvp = report.model;
  if (fabs(vanishing_point.hvp.z()) > 1e-5){
    vanishing_point.hvp /= vanishing_point.hvp.z();
  }
  vanishing_point.vp = report.model.hnormalized();
  vanishing_point.num_inliers = report.support.num_inliers;
  for (size_t i=0; i<report.inlier_mask.size(); i++){
    if (report.inlier_mask[i]){
      vanishing_point.inlier_line_segment_ids.push_back(
          line_segments[i].line_segment_id);
    }
  }

  return vanishing_point;
}

std::vector<LineSegment> ExtractOutlierLineSegments(
    const std::vector<LineSegment>& line_segments,
    const std::vector<int>& inlier_line_segment_ids){

  std::unordered_set<int> inlier_set;
  for (size_t i=0; i<inlier_line_segment_ids.size(); i++){
    inlier_set.emplace(inlier_line_segment_ids[i]);
  }

  std::vector<LineSegment> outlier_line_segments;
  for (size_t i=0; i<line_segments.size(); i++){
    if (inlier_set.count(line_segments[i].line_segment_id) == 0){
      outlier_line_segments.push_back(line_segments[i]);
    }
  }

  return outlier_line_segments;
}

// Multi-Ransac to estimate multiple vanishing points.
std::vector<VanishingPoint> EstimateVanishingPoints(
    const VanishingPointOptions& options,
    const std::vector<LineSegment>& line_segments){
  // Init
  std::vector<VanishingPoint> vanishing_points;
  std::vector<LineSegment> remaining_line_segments = line_segments;

  while (true){
    // If too many vanishing points found already, the next vanishing point
    // will probably noisy or degenerate ones, so stop
    if (vanishing_points.size() >= (size_t)options.max_vanishing_points_num){
      break;
    }

    // If there are too few remaining lines to find a vanishing point for, stop
    if (remaining_line_segments.size() <
        options.min_remaining_lines_ratio * line_segments.size()){
      break;
    }

    VanishingPoint vanishing_point = EstimateVanishingPoint(
        options, remaining_line_segments);

    // If estimation failed, stop
    if (vanishing_point.inlier_line_segment_ids.size() == 0){
      printf("Estimation failed.\nBreak.\n");
      break;
    }

    // If not enough line segment inliers, stop
    if (vanishing_point.inlier_line_segment_ids.size() < 
        (size_t) options.min_num_inliers){
      break;
    }

    // At this stage, it is a valid vanishing point.
    vanishing_points.push_back(vanishing_point);

    // Remove the inliers.
    remaining_line_segments = ExtractOutlierLineSegments(remaining_line_segments,
        vanishing_point.inlier_line_segment_ids);
  }
  
  return vanishing_points;
}

// One vertical vp and multiple horizontal vps.
void DetectVanishingPoints(
    const VanishingPointOptions& options,
    LineSegments& line_segments,
    std::vector<VanishingPoint>& vanishing_points){
  // Next vanishing point id available.
  int vanishing_point_id = 0;

  // Classify lines as vertical or horizontal
  std::vector<int> horizontal_line_segment_ids;
  std::vector<int> vertical_line_segment_ids;
  std::vector<LineSegment> horizontal_line_segments;
  std::vector<LineSegment> vertical_line_segments;
  for (auto& line_segment : line_segments){
    // May be redundant if done in the pre-processing
    line_segment.second.ClassifyOrientation(options.line_orientation_tolerance);
    if (line_segment.second.orientation == LineSegmentOrientation::HORIZONTAL){
      horizontal_line_segment_ids.push_back(line_segment.first);
      horizontal_line_segments.push_back(line_segment.second);
    }
    else if (line_segment.second.orientation == LineSegmentOrientation::VERTICAL){
      vertical_line_segment_ids.push_back(line_segment.first);
      vertical_line_segments.push_back(line_segment.second);
    }
  }

  // Estimate a single vertical vanishing point.
  VanishingPoint vertical_vanishing_point = EstimateVanishingPoint(
      options, vertical_line_segments);
  vertical_vanishing_point.id = vanishing_point_id;
  vertical_vanishing_point.orientation = VanishingPoint::Orientation::VERTICAL;
  vanishing_point_id ++;
  // update line segment class
  for (size_t i=0; i<vertical_vanishing_point.inlier_line_segment_ids.size(); i++){
    int line_segment_id = vertical_vanishing_point.inlier_line_segment_ids[i];
    line_segments[line_segment_id].vanishing_point_id = vertical_vanishing_point.id;
  }
  if (vertical_vanishing_point.inlier_line_segment_ids.size() > 0){
    vanishing_points.push_back(vertical_vanishing_point);
  }

  // Estimate multiple horizontal vanishing points.
  std::vector<VanishingPoint> horizontal_vanishing_points = EstimateVanishingPoints(
      options, horizontal_line_segments);
  for (size_t j=0; j<horizontal_vanishing_points.size(); j++){
    VanishingPoint horizontal_vanishing_point = horizontal_vanishing_points[j];
    
    // Update indices, orientations
    horizontal_vanishing_point.orientation = VanishingPoint::Orientation::HORIZONTAL;
    horizontal_vanishing_point.id = vanishing_point_id;
    vanishing_point_id ++;
    vanishing_points.push_back(horizontal_vanishing_point);

    // update line segment class
    for (size_t i=0; i<horizontal_vanishing_point.inlier_line_segment_ids.size(); i++){
      int line_segment_id = horizontal_vanishing_point.inlier_line_segment_ids[i];
      line_segments[line_segment_id].vanishing_point_id = horizontal_vanishing_point.id;
    }
  }
}

} // namespace goma
