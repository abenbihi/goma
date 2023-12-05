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
// Editor: Assia Benbihi (assia-dot-benbihi-at-cvut-dot-cz)
//                       (abenbihi-at-georgiatech-hyphen-metz-dot-fr)
// Edits:
// Generic LORANSAC with the following edits:
//  - The inliers used in the lo-step can be collected with an error threshold
//    (max_error2) higher than the one used to score the estimated model
//    (max_error) to enable higher tolerance when refitting). 
//  - Allow for estimation constrained with constraints matrix (e.g. from
//    vanishing points).

#ifndef GOMA_OPTIM_LORANSAC_H_
#define GOMA_OPTIM_LORANSAC_H_

#include <cfloat>
#include <random>
#include <stdexcept>
#include <vector>

// colmap includes
#include "optim/random_sampler.h"
#include "optim/ransac.h"
#include "optim/support_measurement.h"
#include "util/alignment.h"
#include "util/logging.h"

namespace goma {

// Implementation of LO-RANSAC (Locally Optimized RANSAC).
//
// "Locally Optimized RANSAC" Ondrej Chum, Jiri Matas, Josef Kittler, DAGM 2003.
template <typename Estimator, typename LocalEstimator,
          typename SupportMeasurer = colmap::InlierSupportMeasurer,
          typename Sampler = colmap::RandomSampler>
class LORANSAC : public colmap::RANSAC<Estimator, SupportMeasurer, Sampler> {
 public:
  using typename colmap::RANSAC<Estimator, SupportMeasurer, Sampler>::Report;

  explicit LORANSAC(const colmap::RANSACOptions& options);

  // Robustly estimate model with RANSAC (RANdom SAmple Consensus).
  //
  // @param X              Independent variables.
  // @param Y              Dependent variables.
  //
  // @return               The report with the results of the estimation.
  Report Estimate(const std::vector<typename Estimator::X_t>& X,
                  const std::vector<typename Estimator::Y_t>& Y);

  // Objects used in RANSAC procedure.
  using colmap::RANSAC<Estimator, SupportMeasurer, Sampler>::estimator;
  LocalEstimator local_estimator;
  using colmap::RANSAC<Estimator, SupportMeasurer, Sampler>::sampler;
  using colmap::RANSAC<Estimator, SupportMeasurer, Sampler>::support_measurer;

  void SetConstraintMatrix(const Eigen::MatrixXd& constraint_matrix){
    constraint_matrix_ = constraint_matrix;
    is_constrained = true;
  }

  // Error threshold used to sample the inliers used in the local estimation.
  // This may be different from the error threshold to define the final set of
  // inliers.
  void SetLocalError(const double& error){
    max_error2 = error;
  }

 private:
  using colmap::RANSAC<Estimator, SupportMeasurer, Sampler>::options_;
  double max_error2; // for when you want to use relaxed set of inliers for the LO-step

  Eigen::MatrixXd constraint_matrix_;
  bool is_constrained;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename Estimator, typename LocalEstimator, typename SupportMeasurer,
          typename Sampler>
LORANSAC<Estimator, LocalEstimator, SupportMeasurer, Sampler>::LORANSAC(
    const colmap::RANSACOptions& options)
    : colmap::RANSAC<Estimator, SupportMeasurer, Sampler>(options),
      max_error2(2*options.max_error),
      is_constrained(false) {}

template <typename Estimator, typename LocalEstimator, typename SupportMeasurer,
          typename Sampler>
typename LORANSAC<Estimator, LocalEstimator, SupportMeasurer, Sampler>::Report
LORANSAC<Estimator, LocalEstimator, SupportMeasurer, Sampler>::Estimate(
    const std::vector<typename Estimator::X_t>& X,
    const std::vector<typename Estimator::Y_t>& Y) {
  CHECK_EQ(X.size(), Y.size());

  const size_t num_samples = X.size();

  typename colmap::RANSAC<Estimator, SupportMeasurer, Sampler>::Report report;
  report.success = false;
  report.num_trials = 0;

  if (num_samples < Estimator::kMinNumSamples) {
    return report;
  }

  typename Estimator::M_t best_sample; // best model estimated from sample
  typename SupportMeasurer::Support best_support; // associated with best_sample
  const double max_residual = options_.max_error * options_.max_error;

  typename Estimator::M_t best_model; // best lo-ransac model
  typename SupportMeasurer::Support best_local_support; // associated with best_model
  const double max_local_residual = max_error2 * max_error2;

  bool best_model_is_local = false;

  bool abort = false;

  std::vector<double> residuals(num_samples);
  std::vector<double> best_local_residuals;

  std::vector<typename LocalEstimator::X_t> X_inlier;
  std::vector<typename LocalEstimator::Y_t> Y_inlier;

  std::vector<typename Estimator::X_t> X_rand(Estimator::kMinNumSamples);
  std::vector<typename Estimator::Y_t> Y_rand(Estimator::kMinNumSamples);

  sampler.Initialize(num_samples);

  size_t max_num_trials = options_.max_num_trials;
  max_num_trials = std::min<size_t>(max_num_trials, sampler.MaxNumSamples());
  size_t dyn_max_num_trials = max_num_trials;

  for (report.num_trials = 0; report.num_trials < max_num_trials;
      ++report.num_trials) {
    if (abort) {
      report.num_trials += 1;
      break;
    }

    sampler.SampleXY(X, Y, &X_rand, &Y_rand);

    // Estimate model for current subset.
    std::vector<typename Estimator::M_t> sample_models;
    if (is_constrained) { 
      sample_models = estimator.EstimateWithConstraints(constraint_matrix_,
          X_rand, Y_rand);
    }
    else{
      sample_models = estimator.Estimate(X_rand, Y_rand);
    }

    // Iterate through all estimated models
    for (const auto& sample_model : sample_models) {
      estimator.Residuals(X, Y, sample_model, &residuals);
      CHECK_EQ(residuals.size(), X.size());

      const auto support = support_measurer.Evaluate(residuals, max_residual);

      // Do local optimization if better than all previous subsets.
      if (support_measurer.Compare(support, best_support)) {
        best_support = support;
        best_sample = sample_model;
        best_model_is_local = false;

        // Estimate locally optimized model from inliers.
        if (support.num_inliers > Estimator::kMinNumSamples &&
            support.num_inliers >= LocalEstimator::kMinNumSamples) {
          // Recursive local optimization to expand inlier set.
          const size_t kMaxNumLocalTrials = 10;
          for (size_t local_num_trials = 0;
              local_num_trials < kMaxNumLocalTrials; ++local_num_trials) {

            X_inlier.clear();
            Y_inlier.clear();

            // Sample inliers with higher tolerance than when evaluating the
            // sample.
            const auto local_support = support_measurer.Evaluate(residuals, max_local_residual);
            X_inlier.reserve(local_support.num_inliers);
            Y_inlier.reserve(local_support.num_inliers);
            for (size_t i = 0; i < residuals.size(); ++i) {
              if (residuals[i] <= max_local_residual) {
                X_inlier.push_back(X[i]);
                Y_inlier.push_back(Y[i]);
              }
            }

            std::vector<typename LocalEstimator::M_t> local_models;
            if (is_constrained) {
              local_models = local_estimator.EstimateWithConstraints(
                  constraint_matrix_, X_inlier, Y_inlier);
            }
            else{
              local_models = local_estimator.Estimate(X_inlier, Y_inlier);
            }

            const size_t prev_best_num_inliers = best_support.num_inliers;

            for (const auto& local_model : local_models) {
              local_estimator.Residuals(X, Y, local_model, &residuals);
              CHECK_EQ(residuals.size(), X.size());

              const auto local_support =
                support_measurer.Evaluate(residuals, max_residual);

              // Check if non-locally optimized model is better.
              if ((support_measurer.Compare(local_support, support)) && 
                  (support_measurer.Compare(local_support, best_local_support))) {
                best_local_support = local_support;
                best_model = local_model;
                best_model_is_local = true;
                std::swap(residuals, best_local_residuals);
              }
            }

            // Only continue recursive local optimization, if the inlier set
            // size increased and we thus have a chance to further improve.
            if (best_support.num_inliers <= prev_best_num_inliers) {
              break;
            }

            // Swap back the residuals, so we can extract the best inlier
            // set in the next recursion of local optimization.
            std::swap(residuals, best_local_residuals);
          }
        }

        dyn_max_num_trials =
          colmap::RANSAC<Estimator, SupportMeasurer, Sampler>::ComputeNumTrials(
              best_support.num_inliers, num_samples, options_.confidence,
              options_.dyn_num_trials_multiplier);
      }

      if (report.num_trials >= dyn_max_num_trials &&
          report.num_trials >= options_.min_num_trials) {
        abort = true;
        break;
      }
    }
  }

  // Check which is best: best_sample (i.e. the RANSAC model) or best_model
  // (the LO-RANSAC refinement model)
  if (best_support.num_inliers > best_local_support.num_inliers){
    report.support = best_support;
    report.model = best_sample;
  }
  else {
    best_model_is_local = true;
    report.support = best_local_support;
    report.model = best_model;
  }

  // No valid model was found
  if (report.support.num_inliers < estimator.kMinNumSamples) {
    return report;
  }

  report.success = true;

  // Determine inlier mask. Note that this calculates the residuals for the
  // best model twice, but saves to copy and fill the inlier mask for each
  // evaluated model. Some benchmarking revealed that this approach is faster.

  if (best_model_is_local) {
    local_estimator.Residuals(X, Y, report.model, &residuals);
  } else {
    estimator.Residuals(X, Y, report.model, &residuals);
  }

  CHECK_EQ(residuals.size(), X.size());

  report.inlier_mask.resize(num_samples);
  for (size_t i = 0; i < residuals.size(); ++i) {
    if (residuals[i] <= max_residual) {
      report.inlier_mask[i] = true;
    } else {
      report.inlier_mask[i] = false;
    }
  }

  return report;
}


}  // namespace goma

#endif  // GOMA_LORANSAC_H_
