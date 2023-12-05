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

#include <Eigen/Core>
#include <Eigen/Dense>

#include "goma/geometry/vanishing_point.h"

namespace goma {

double VanishingPointResidue(const Eigen::Vector2d& start,
    const Eigen::Vector2d& end, const Eigen::Vector3d& vanishing_point){
  Eigen::Vector3d midpoint = (0.5 * (start + end)).homogeneous();

  // Check that the vanishing point does not unluckily falls on the
  // midpoint. 
  if (fabs(vanishing_point(2))>1e-3){
    Eigen::Vector2d vp = vanishing_point.hnormalized();
    if ((midpoint.hnormalized() - vp).norm() < 5){
      return 1e4;
    }
  }

  Eigen::Vector3d connecting_line = midpoint.cross(
      vanishing_point);

  Eigen::Vector2d u = end - start;
  assert(u.norm() > 1e-6);
  u /= u.norm();
  
  // The original code computes the distance between the connecting
  // line and line segment end, which can cause the distance to depend on the
  // line segment length.
  // For example, small deviations between the direction of the connecting
  // line and the line segment direction will cause a larger distance for
  // long line segment than for short line segments.
  // By using a point on the line segment at a fixed distance from the line segment's midpoint,
  // the distance with the connecting line does not depend on the line
  // segment's length anymore.
  // Note: this dependence can be advantageous depending on your usecase.
  Eigen::Vector2d fix_distance_end = midpoint.hnormalized() + 100 * u;

  const double signed_distance =
    connecting_line.dot(fix_distance_end.homogeneous()) /
    connecting_line.head<2>().norm();

  double residue = signed_distance * signed_distance;

  return residue;
}

} // namespace goma
