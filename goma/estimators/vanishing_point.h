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
//                       
// Edits:
// - Adds a local estimator to enable LO-RANSAC for vanishing point estimation.
// - Updates the residual cost function to catch edge cases:
//   - When the vanishing point falls on the midpoint of a line segment.
//   - The residual cost does not depend on the line segment length
//     anymore.

#ifndef GOMA_ESTIMATORS_VANISHING_POINT_H
#define GOMA_ESTIMATORS_VANISHING_POINT_H

// colmap includes
#include <estimators/affine_transform.h>
#include <estimators/coordinate_frame.h>
#include <estimators/utils.h>

#include "goma/geometry/vanishing_point.h"
#include "goma/scene/line_segment.h"
#include "goma/scene/vanishing_point.h"

namespace goma {

struct VanishingPointEstimator {
  // The line segments.
  typedef LineSegment X_t;
  // The line representation of the segments.
  typedef Eigen::Vector3d Y_t;
  // The vanishing point.
  typedef Eigen::Vector3d M_t;

  // The minimum number of samples needed to estimate a model.
  static const int kMinNumSamples = 2;

  // Estimate the vanishing point from at least two line segments.
  static std::vector<M_t> Estimate(const std::vector<X_t>& line_segments,
                                   const std::vector<Y_t>& lines) {
    CHECK_EQ(line_segments.size(), 2);
    CHECK_EQ(lines.size(), 2);
    return {lines[0].cross(lines[1])};
  }
  
  // This is just to agree with the global api. There is no constrained
  // optimization for vanishing points.
  static std::vector<M_t> EstimateWithConstraints(
      const Eigen::MatrixXd constraint_matrix,
      const std::vector<X_t>& line_segments,
      const std::vector<Y_t>& lines){
    printf("Warning: no constrained optimization available for vanishing point.\n");
    printf("Default to the standard estimation.\n");
    std::vector<M_t> models = Estimate(line_segments, lines);
    return models;
  }

  // Calculate the squared distance of each line segment's end point to the line
  // connecting the vanishing point and the midpoint of the line segment.
  static void Residuals(const std::vector<X_t>& line_segments,
                        const std::vector<Y_t>& lines,
                        const M_t& vanishing_point,
                        std::vector<double>* residuals) {
    residuals->resize(line_segments.size());

    // Check if vanishing point is at infinity.
    if (vanishing_point[2] == 0) {
      std::fill(residuals->begin(), residuals->end(),
                std::numeric_limits<double>::max());
      return;
    }

    for (size_t i = 0; i < lines.size(); ++i) {
      (*residuals)[i] = VanishingPointResidue(line_segments[i].start,
          line_segments[i].end, vanishing_point);
    }
  }
};

struct VanishingPointLocalEstimator {
  // The line segments.
  typedef LineSegment X_t;
  // The line representation of the segments.
  typedef Eigen::Vector3d Y_t;
  // The vanishing point.
  typedef Eigen::Vector3d M_t;

  // The minimum number of samples needed to estimate a model.
  static const int kMinNumSamples = 2;

  // Estimate the vanishing points from more than 2 line segments.
  static std::vector<M_t> Estimate(const std::vector<X_t>& line_segments,
                                   const std::vector<Y_t>& lines);

  static std::vector<M_t> EstimateWithConstraints(
      const Eigen::MatrixXd constraint_matrix,
      const std::vector<X_t>& line_segments,
      const std::vector<Y_t>& lines);

  // Calculate the squared distance of each line segment's end point to the line
  // connecting the vanishing point and the midpoint of the line segment.
  static void Residuals(const std::vector<X_t>& line_segments,
                        const std::vector<Y_t>& lines,
                        const M_t& vanishing_point,
                        std::vector<double>* residuals);
};

} // namespace goma

#endif // GOMA_ESTIMATORS_VANISHING_POINT_SOLVER_H
