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
// Edits: Allows for homography estimation between 3D planes from point matches
// and vanishing direction constraints.

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

// colmap includes
#include "base/projection.h"
#include "estimators/utils.h"
#include "util/logging.h"

#include "goma/estimators/homography_matrix.h"

namespace goma {
  
Eigen::MatrixXd HomographyMatrixEstimator::EstimateVanishingPointConstraints(
    const std::vector<X_t>& kernel_points1, const std::vector<Y_t>& kernel_points2) {
  CHECK_EQ(kernel_points1.size(), kernel_points2.size());

  // the vanishing points satisfy the homography so A_vp * h = 0
  // This means that h lies in the null space of A_vp and can be expressed as
  // a linear combination of a base of its null space.
  // Let's find this null space
  // Setup constraint matrix.
  const size_t kernel_N = kernel_points1.size();
  Eigen::Matrix<double, Eigen::Dynamic, 9> A_vp = Eigen::MatrixXd::Zero(2 * kernel_N, 9);
  for (size_t i=0, j=kernel_N; i<kernel_N; ++i, ++j){
    const double s_0 = kernel_points1[i](0);
    const double s_1 = kernel_points1[i](1);
    const double d_0 = kernel_points2[i](0);
    const double d_1 = kernel_points2[i](1);

    A_vp(i, 0) = -s_0;
    A_vp(i, 1) = -s_1;
    A_vp(i, 2) = -1;
    A_vp(i, 6) = s_0 * d_0;
    A_vp(i, 7) = s_1 * d_0;
    A_vp(i, 8) = d_0;

    A_vp(j, 3) = -s_0;
    A_vp(j, 4) = -s_1;
    A_vp(j, 5) = -1;
    A_vp(j, 6) = s_0 * d_1;
    A_vp(j, 7) = s_1 * d_1;
    A_vp(j, 8) = d_1;
  }
  // Solve for the nullspace of the constraint matrix.
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd_A_vp(A_vp, Eigen::ComputeFullV);
  Eigen::MatrixXd B = svd_A_vp.matrixV().rightCols<5>(); // base of the nullspace of A_vp

  return B;
}

std::vector<HomographyMatrixEstimator::M_t> 
HomographyMatrixEstimator::EstimateWithConstraints(
    const Eigen::MatrixXd& constraint_matrix,
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
  //CHECK_EQ(kernel_points1.size(), kernel_points2.size());
  CHECK_EQ(points1.size(), points2.size());

  // We now know that h = B * alpha with alpha in R^5, B in R^{9,5}
  // We have four box correspondences, and we look for alpha that minimizes
  // A_box*h = 0 <=> A_box * B * alpha = 0
  // Setup constraint matrix.
  const size_t N = points1.size();
  //std::cout << "N: " << N << std::endl;
  //assert(N>0);
  // TODO: for now, do not normalize points and see what happens
  std::vector<X_t> normed_points1 = points1;
  std::vector<Y_t> normed_points2 = points2;
  Eigen::Matrix3d points1_norm_matrix = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d points2_norm_matrix = Eigen::Matrix3d::Identity();

  // Setup constraint matrix.
  Eigen::Matrix<double, Eigen::Dynamic, 9> A_box = Eigen::MatrixXd::Zero(2*N, 9);
  //std::cout << "A_box (before)" << std::endl << A_box << std::endl;

  for (size_t i = 0, j = N; i < points1.size(); ++i, ++j) {
    //std::cout << i << std::endl;
    const double s_0 = normed_points1[i](0);
    const double s_1 = normed_points1[i](1);
    const double d_0 = normed_points2[i](0);
    const double d_1 = normed_points2[i](1);

    A_box(i, 0) = -s_0;
    A_box(i, 1) = -s_1;
    A_box(i, 2) = -1;
    A_box(i, 6) = s_0 * d_0;
    A_box(i, 7) = s_1 * d_0;
    A_box(i, 8) = d_0;

    A_box(j, 3) = -s_0;
    A_box(j, 4) = -s_1;
    A_box(j, 5) = -1;
    A_box(j, 6) = s_0 * d_1;
    A_box(j, 7) = s_1 * d_1;
    A_box(j, 8) = d_1;
  }
  //std::cout << "A_box" << std::endl << A_box << std::endl;
  //assert(A_box.cols() == B.rows());
  Eigen::MatrixXd A = A_box * constraint_matrix;
  //std::cout << "A" << std::endl << A << std::endl;

  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 5>> svd(
      A, Eigen::ComputeFullV);
    
  const Eigen::VectorXd alpha = svd.matrixV().col(4);
  Eigen::MatrixXd h = constraint_matrix * alpha;
  Eigen::Map<const Eigen::Matrix3d> H_t(h.data());
  const std::vector<M_t> models = {points2_norm_matrix.inverse() *
                                   H_t.transpose() * points1_norm_matrix};

  return models;
}

std::vector<HomographyMatrixEstimator::M_t> HomographyMatrixEstimator::Estimate(
    const std::vector<X_t>& kernel_points1, const std::vector<Y_t>& kernel_points2, 
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
  CHECK_EQ(kernel_points1.size(), kernel_points2.size());
  CHECK_EQ(points1.size(), points2.size());

  Eigen::MatrixXd B = EstimateVanishingPointConstraints(kernel_points1, kernel_points2);
  const std::vector<M_t> models = EstimateWithConstraints(B, points1, points2);
  return models;
}

std::vector<HomographyMatrixEstimator::M_t> HomographyMatrixEstimator::Estimate(
    const std::vector<X_t>& points1, const std::vector<Y_t>& points2) {
  CHECK_EQ(points1.size(), points2.size());

  const size_t N = points1.size();

  // Center and normalize image points for better numerical stability.
  std::vector<X_t> normed_points1;
  std::vector<Y_t> normed_points2;
  Eigen::Matrix3d points1_norm_matrix;
  Eigen::Matrix3d points2_norm_matrix;
  colmap::CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
  colmap::CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

  // Setup constraint matrix.
  Eigen::Matrix<double, Eigen::Dynamic, 9> A = Eigen::MatrixXd::Zero(2 * N, 9);

  for (size_t i = 0, j = N; i < points1.size(); ++i, ++j) {
    const double s_0 = normed_points1[i](0);
    const double s_1 = normed_points1[i](1);
    const double d_0 = normed_points2[i](0);
    const double d_1 = normed_points2[i](1);

    A(i, 0) = -s_0;
    A(i, 1) = -s_1;
    A(i, 2) = -1;
    A(i, 6) = s_0 * d_0;
    A(i, 7) = s_1 * d_0;
    A(i, 8) = d_0;

    A(j, 3) = -s_0;
    A(j, 4) = -s_1;
    A(j, 5) = -1;
    A(j, 6) = s_0 * d_1;
    A(j, 7) = s_1 * d_1;
    A(j, 8) = d_1;
  }

  // Solve for the nullspace of the constraint matrix.
  Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(
      A, Eigen::ComputeFullV);

  const Eigen::VectorXd nullspace = svd.matrixV().col(8);
  Eigen::Map<const Eigen::Matrix3d> H_t(nullspace.data());

  const std::vector<M_t> models = {points2_norm_matrix.inverse() *
                                   H_t.transpose() * points1_norm_matrix};
  return models;
}

void HomographyMatrixEstimator::Residuals(const std::vector<X_t>& points1,
                                          const std::vector<Y_t>& points2,
                                          const M_t& H,
                                          std::vector<double>* residuals) {
  CHECK_EQ(points1.size(), points2.size());

  residuals->resize(points1.size());

  // Note that this code might not be as nice as Eigen expressions,
  // but it is significantly faster in various tests.

  const double H_00 = H(0, 0);
  const double H_01 = H(0, 1);
  const double H_02 = H(0, 2);
  const double H_10 = H(1, 0);
  const double H_11 = H(1, 1);
  const double H_12 = H(1, 2);
  const double H_20 = H(2, 0);
  const double H_21 = H(2, 1);
  const double H_22 = H(2, 2);

  for (size_t i = 0; i < points1.size(); ++i) {
    const double s_0 = points1[i](0);
    const double s_1 = points1[i](1);
    const double d_0 = points2[i](0);
    const double d_1 = points2[i](1);

    const double pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
    const double pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
    const double pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

    const double inv_pd_2 = 1.0 / pd_2;
    const double dd_0 = d_0 - pd_0 * inv_pd_2;
    const double dd_1 = d_1 - pd_1 * inv_pd_2;

    (*residuals)[i] = dd_0 * dd_0 + dd_1 * dd_1;
  }
}

}  // namespace goma

