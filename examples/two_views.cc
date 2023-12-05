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
//
// Given two images, their local features and matching planes, computes the
// homography relating the matching planes from point matching and vanishing
// direction constraints.

#include "base/camera.h"
#include "util/misc.h"

#include "goma/feature/lsd.h"
#include "goma/feature/types.h"
#include "goma/feature/vanishing_point.h"
#include "goma/estimators/homography_matrix.h"
#include "goma/optim/loransac.h"
#include "goma/scene/scene.h"
#include "goma/util/loader.h"
#include "goma/util/reader.h"
#include "goma/util/option_manager.h"

#include "examples/image.h"

namespace goma {

typedef std::pair<int, int> PairIdx;

typedef std::map<PairIdx, std::vector<PairIdx> > PairwiseMatchIndices;

void WriteHomographiesAndInlierMatches(const std::string& path,
    const std::vector<PairIdx>& plane_matches,
    const std::vector<Eigen::Matrix3d>& homographies,
    const std::vector<std::vector<PairIdx> >& homography_inlier_matches){

  std::ofstream file(path, std::ios::trunc);
  CHECK(file.is_open()) << path;

  // Ensure that we don't loose any precision by storing in text.
  file.precision(17);
  
  // header: num plane matches
  std::ostringstream line;
  std::string line_string;
  line << plane_matches.size();
  line_string = line.str();
  file << line_string << std::endl;
  line.str("");
  line.clear();
  
  // body1: plane matches and relating homography
  for (size_t i=0; i<plane_matches.size(); i++){
    int plane_idx1 = plane_matches[i].first;
    int plane_idx2 = plane_matches[i].second;

    Eigen::Matrix3d H = homographies[i];

    line << plane_idx1 << " ";
    line << plane_idx2 << " ";
    line << H(0,0) << " ";
    line << H(0,1) << " ";
    line << H(0,2) << " ";
    line << H(1,0) << " ";
    line << H(1,1) << " ";
    line << H(1,2) << " ";
    line << H(2,0) << " ";
    line << H(2,1) << " ";
    line << H(2,2);

    line_string = line.str();
    file << line_string << std::endl;
    line.str("");
    line.clear();
  }

  // body2: point matches

  // Count num point matches
  size_t num_point_matches = 0;
  for (size_t i=0; i<plane_matches.size(); i++){
    num_point_matches += homography_inlier_matches[i].size();
  }
  line << "POINT_MATCHES " << num_point_matches;
  line_string = line.str();
  file << line_string << std::endl;
  line.str("");
  line.clear();

  for (size_t i=0; i<plane_matches.size(); i++){
    int plane_idx1 = plane_matches[i].first;
    int plane_idx2 = plane_matches[i].second;

    for (size_t j=0; j<homography_inlier_matches[i].size(); j++){
      int point_idx1 = homography_inlier_matches[i][j].first;
      int point_idx2 = homography_inlier_matches[i][j].second;
      line << plane_idx1 << " " << plane_idx2 << " " ;
      line << point_idx1 << " " << point_idx2;

      line_string = line.str();
      file << line_string << std::endl;
      line.str("");
      line.clear();
    }
  }
}

Image ReadImage(const ReaderOptions& reader, const std::string& image_name){ 
  Image image;

  // Read image vanishing points.
  image.vanishing_points = ReadVanishingPoints(
      colmap::JoinPaths(reader.vanishing_point_path, image_name + ".txt"));

  // Read image planes.
  image.planes = ReadPlanes(
    colmap::JoinPaths(reader.plane_path, image_name + ".txt"));

  std::string base_path;
  std::string extension;
  colmap::SplitFileExtension(image_name, &base_path, &extension);
  std::string feature_path = colmap::JoinPaths(reader.feature_path,
      base_path) + "_pts.txt";
  image.points = ReadPoints(feature_path);

  return image;
}

PairwiseMatchIndices ReadPlaneWisePointMatches(const std::string& match_path,
    const std::string& image_name1, const std::string& image_name2){
  std::string path = GenMatchesPath(match_path, image_name1, image_name2);
  std::cout << path << std::endl;

  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  std::getline(file, line);
  std::stringstream header_line_stream(line);

  std::getline(header_line_stream >> std::ws, item, ' ');
  const size_t num_matches = std::stoul(item);

  PairwiseMatchIndices plane_matches;

  for (size_t i=0; i<num_matches; i++){
    std::getline(file, line);
    std::stringstream feature_line_stream(line);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    int plane_id1 = std::stoul(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    int plane_id2 = std::stoul(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    int point_id1 = std::stoul(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    int point_id2 = std::stoul(item);

    plane_matches[PairIdx(plane_id1, plane_id2)].push_back(
        PairIdx(point_id1, point_id2));
  }

  return plane_matches;
}

int RunPlanewiseHomographyEstimation(int argc, char* argv[]){
  // Parse options
  OptionManager options;
  options.AddFeatureMatchingOptions();
  options.AddReaderOptions();
  options.Parse(argc, argv);

  FeatureMatchingOptions feature_match_options = *options.feature_matching;
  ReaderOptions reader = *options.reader;
  options.Write(reader.config_path);

  Scene scene(reader);

  // Read image keypoints
  std::string image_name1 = scene.images[reader.image_id1].image_name;
  std::string image_name2 = scene.images[reader.image_id2].image_name;
  std::cout << image_name1 << " X " << image_name2 << std::endl;
  Image image1 = ReadImage(reader, image_name1);
  Image image2 = ReadImage(reader, image_name2);

  // Read feature matches and their support plane.
  PairwiseMatchIndices plane_point_matches = ReadPlaneWisePointMatches(
      reader.match_path, image_name1, image_name2);
  
  // Output
  std::vector<Eigen::Matrix3d> homographies;
  std::vector<std::vector<PairIdx> > homography_inlier_matches;
  std::vector<PairIdx> plane_matches;

  for (PairwiseMatchIndices::iterator it=plane_point_matches.begin();
      it!=plane_point_matches.end(); it++){
    int plane_idx1 = (it->first).first;
    int plane_idx2 = (it->first).second;
    printf("Planes %d X %d\n", plane_idx1, plane_idx2);
    plane_matches.push_back(PairIdx(plane_idx1, plane_idx2));

    std::vector<PairIdx> point_match_indices = it->second;

    // Select matching points between the current plane pair.
    std::vector<Eigen::Vector2d> matched_points1(point_match_indices.size());
    std::vector<Eigen::Vector2d> matched_points2(point_match_indices.size());
    for (size_t i = 0; i < point_match_indices.size(); ++i) {
      matched_points1[i] = image1.points[point_match_indices[i].first];
      matched_points2[i] = image2.points[point_match_indices[i].second];
    }

    // Format vanishing point of each plane
    std::vector<Eigen::Vector2d> vanishing_points1(2);
    vanishing_points1[0] = image1.planes[plane_idx1].horizontal_vanishing_point;
    vanishing_points1[1] = image1.planes[plane_idx1].vertical_vanishing_point;
    std::vector<Eigen::Vector2d> vanishing_points2(2);
    vanishing_points2[0] = image2.planes[plane_idx2].horizontal_vanishing_point;
    vanishing_points2[1] = image2.planes[plane_idx2].vertical_vanishing_point;

    // Estimate homography constrained with vanishing direction from matching
    // points.
    colmap::RANSACOptions ransac_options;
    ransac_options.max_error = feature_match_options.max_error;
    ransac_options.min_inlier_ratio = feature_match_options.min_inlier_ratio;
    ransac_options.confidence = feature_match_options.confidence;
    ransac_options.min_num_trials = feature_match_options.min_num_trials;
    ransac_options.max_num_trials = feature_match_options.max_num_trials;

    Eigen::MatrixXd constraint_matrix = 
      goma::HomographyMatrixEstimator::EstimateVanishingPointConstraints(
          vanishing_points1, vanishing_points2);

    goma::LORANSAC<goma::HomographyMatrixEstimator, 
      goma::HomographyMatrixEstimator> H_ransac(ransac_options);

    H_ransac.SetConstraintMatrix(constraint_matrix);
    const auto H_report = H_ransac.Estimate(matched_points1, matched_points2);

    // Store resulting homography and inliers
    std::vector<PairIdx> inlier_point_match_indices;
  
    // If no valid homography, store empty results
    if (!H_report.success){
      printf("Oh no! No valid homography.\n");
      homographies.push_back(Eigen::Matrix3d::Zero());
      homography_inlier_matches.push_back(inlier_point_match_indices);
      continue;
    }
  
    // Extract inliers
    for (size_t i=0; i<point_match_indices.size(); i++){
      if (H_report.inlier_mask[i]){
        inlier_point_match_indices.push_back(point_match_indices[i]);
      }
    }
    printf("# inliers: %ld\n", inlier_point_match_indices.size());

    homographies.push_back(H_report.model);
    homography_inlier_matches.push_back(inlier_point_match_indices);
  }

  // Write homographies and inliers to files.
  std::string output_path = GenMatchesPath(reader.output_path,
      image_name1, image_name2);
  WriteHomographiesAndInlierMatches(output_path, plane_matches, homographies,
      homography_inlier_matches);
  printf("Wrote homographies to %s\n", output_path.c_str());

  return 0;
}

} // namespace goma
