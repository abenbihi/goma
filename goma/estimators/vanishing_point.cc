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

#include "goma/estimators/vanishing_point.h"

namespace goma { 

std::vector<VanishingPointLocalEstimator::M_t> VanishingPointLocalEstimator::Estimate(
    const std::vector<VanishingPointLocalEstimator::X_t>& line_segments,
    const std::vector<VanishingPointLocalEstimator::Y_t>& lines) {
  // The vanishing point X must belong to all lines (l_i) i.e. l_i^T * X = 0
  const size_t N = lines.size();

  Eigen::Matrix<double, Eigen::Dynamic, 3> A = Eigen::MatrixXd::Zero(N,3);
  for (size_t i=0; i<lines.size(); i++){
    A(i,0) = lines[i](0);
    A(i,1) = lines[i](1);
    A(i,2) = lines[i](2);
  }
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 3>> svd(A, Eigen::ComputeFullV);

  const std::vector<M_t> models = {svd.matrixV().col(2)}; 

  return models;
}

// Calculate the squared distance of each line segment's end point to the line
// connecting the vanishing point and the midpoint of the line segment.
void VanishingPointLocalEstimator::Residuals(
    const std::vector<VanishingPointLocalEstimator::X_t>& line_segments,
    const std::vector<VanishingPointLocalEstimator::Y_t>& lines,
    const VanishingPointLocalEstimator::M_t& vanishing_point,
    std::vector<double>* residuals) {
  residuals->resize(line_segments.size());

  // Check if vanishing point is at infinity.
  if (vanishing_point[2] == 0) {
    std::fill(residuals->begin(), residuals->end(),
        std::numeric_limits<double>::max());
    return;
  }

  for (size_t i = 0; i < line_segments.size(); ++i) {
    (*residuals)[i] = VanishingPointResidue(line_segments[i].start,
        line_segments[i].end, vanishing_point);;
  }
}

// This is just to agree with the global api. There is no constrained
// optimization for vanishing points.
std::vector<VanishingPointLocalEstimator::M_t> 
  VanishingPointLocalEstimator::EstimateWithConstraints(
      const Eigen::MatrixXd constraint_matrix,
      const std::vector<VanishingPointLocalEstimator::X_t>& line_segments,
      const std::vector<VanishingPointLocalEstimator::Y_t>& lines){
  printf("Warning: no constrained optimization available for vanishing point.\n");
  printf("Default to the standard estimation.\n");
  std::vector<M_t> models = Estimate(line_segments, lines);
  return models;
}

} // namespace goma
