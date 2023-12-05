// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
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
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
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
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)
//
// Editor: Assia Benbihi (abenbihi-at-georgiatech-hyphen-metz-dot-fr)
//                       (assia-dot-benbihi-at-cvut-dot-cz)
// Edits:
//  - New line segment struct
//  - Update the prototype of colmap::ClassifyLineSegmentOrientations to
//    accomdate the new struct
//  - Preprocessing functionalities on the line segments

#ifndef GOMA_IMAGE_LINE_SEGMENT_H
#define GOMA_IMAGE_LINE_SEGMENT_H

#include <unordered_map>

#include <Eigen/Core>

#include <base/camera.h>
#include <base/line.h>
#include "util/alignment.h"

namespace goma {

enum class LineSegmentOrientation {
  HORIZONTAL = 1,
  VERTICAL = -1,
  UNDEFINED = 0,
};

struct LineSegment {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Endpoints
  Eigen::Vector2d start;
  Eigen::Vector2d end;

  // Line segment confidence
  float score;

  int line_segment_id;
 
  // Whether this is a valid line segment or not.
  bool registered;

  LineSegmentOrientation orientation;

  // The identifier of the vanishing point to which this line belong to.
  int vanishing_point_id;

  LineSegment()
    : score(-1),
      line_segment_id(-1),
      registered(false),
      orientation(LineSegmentOrientation::UNDEFINED),
      vanishing_point_id(-1) {}

  void UndistortLineSegment(const colmap::Camera& distorted_camera, 
      const colmap::Camera& undistorted_camera){
    Eigen::Vector2d new_start = undistorted_camera.WorldToImage(
        distorted_camera.ImageToWorld(start));
    Eigen::Vector2d new_end = undistorted_camera.WorldToImage(
        distorted_camera.ImageToWorld(end));

    start = new_start;
    end = new_end;
  }

  // Colmap implementation.
  void ClassifyOrientation(const double line_orientation_tolerance){
    const Eigen::Vector2d direction = (end - start).normalized();
    if (std::abs(direction.x()) + line_orientation_tolerance > 1) {
      orientation = LineSegmentOrientation::HORIZONTAL;
    } else if (std::abs(direction.y()) + line_orientation_tolerance > 1) {
      orientation = LineSegmentOrientation::VERTICAL;
    } else {
      orientation = LineSegmentOrientation::UNDEFINED;
    }
  }
};

typedef EIGEN_STL_UMAP(int, LineSegment) LineSegments;

void ClassifyLineSegmentOrientations(
  const float tolerance,
  std::vector<LineSegment>& line_segments);

void ClassifyLineSegmentOrientations(const float tolerance,
    LineSegments& line_segments);

void FilterLineSegments(const float score, const float min_line_segment_size, 
    LineSegments& line_segments);

} // namespace goma

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(goma::LineSegment)
#endif
