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

#ifndef GOMA_SCENE_VANISHING_POINT_H
#define GOMA_SCENE_VANISHING_POINT_H

#include <string>
#include <climits>
#include <vector>

#include <glog/logging.h>

#include <Eigen/Core>

namespace goma {

  // Estimation options.
  struct VanishingPointOptions {
    // Minimum number of inliers for a vanishing point to be considered as
    // verified.
    int min_num_inliers = 10;

    // For multiple vanishing points estimation, minimum ratio of remaining
    // lines needed to derive a new vanishing point. For online filtering of
    // noisy line segments.
    double min_remaining_lines_ratio = 0.2;

    // Minimum line segment size.
    double min_line_segment_size = 20;

    // Minimum line segment score.
    double min_line_segment_score = 0.0;

    // The tolerance for classifying lines into horizontal/vertical.
    double line_orientation_tolerance = 0.2;

    // Maximum number of vanishing points to find.
    int max_vanishing_points_num = 100;

    // Options used to robustly estimate the vanishing point.
    // Maximum pixel distance between the line segment end point to the line that goes
    // through the estimated vanishing point and the midpoint of the line
    // segment.
    double max_error = 1;

    // Vanishing point residue below which a line segment is considered to
    // re-estimate the vanishing point.
    double max_error2 = 1.1;

    // Confidence threshold for geometric verification.
    double confidence = 0.999;

    // Minimum/maximum number of RANSAC iterations. Note that this option
    // overrules the min_inlier_ratio option.
    int min_num_trials = 100;
    int max_num_trials = 10000;

    // A priori assumed minimum inlier ratio, which determines the maximum
    // number of iterations.
    double min_inlier_ratio = 0.25;

    // Whether to undistort the image and the line segments. Set to 1 to
    // undistort.
    int undistort = 0;

    bool Check() const{
      CHECK_GE(min_num_inliers, 0);
      CHECK_GE(min_remaining_lines_ratio, 0.0);
      CHECK_LE(min_remaining_lines_ratio, 0.9);

      //ransac_options.Check();
      return true;
    }
  };

  struct VanishingPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      enum Orientation {
        HORIZONTAL = 1,
        VERTICAL = -1,
        UNKNOWN = 0,
      };

    // The configuration of the estimated two-view geometry.
    enum ConfigurationType {
      UNDEFINED = 0,
      // Degenerate configuration (e.g., no overlap or not enough inliers).
      DEGENERATE = 1,
    };

    // Identifier of the vanishing point.
    int id;

    // One of `ConfigurationType`.
    int config;

    // Estimated vanishing point.
    Eigen::Vector2d vp;

    // Estimated vanishing point (homogeneous coordinates).
    Eigen::Vector3d hvp;

    // One of the `OrientationType`.
    Orientation orientation;

    // LineSegment ids of inlier line segments.
    std::vector<int> inlier_line_segment_ids;
    //std::vector<size_t> inlier_line_segment_indices;

    // Mask of inlier line segments of the configuration.
    std::vector<char> inlier_mask;

    // Number of inliers.
    int num_inliers;

    VanishingPoint():
      id(-1),
      config(ConfigurationType::UNDEFINED),
      vp(Eigen::Vector2d::Zero()),
      orientation(Orientation::UNKNOWN),
      num_inliers(-1){}
  };

} // namespace goma

//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(goma::VanishingPoint)

#endif // GOMA_ESTIMATORS_VANISHING_POINT_H
