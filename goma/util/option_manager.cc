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
// Edits: Parameters and options registrations for the options specific to
// goma. 

#include "goma/util/option_manager.h"

#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "util/misc.h"

#include "goma/feature/types.h"
#include "goma/scene/vanishing_point.h"
#include "goma/util/reader.h"

namespace config = boost::program_options;

namespace goma {

OptionManager::OptionManager(){
  reader.reset(new ReaderOptions());
  vanishing_point.reset(new VanishingPointOptions());
  feature_matching.reset(new FeatureMatchingOptions());
  
  Reset();
  
  desc_->add_options()("help,h", "");
  
  AddLogOptions();
}

void OptionManager::AddLogOptions() {
  if (added_log_options_) {
    return;
  }
  added_log_options_ = true;

  AddAndRegisterDefaultOption("log_to_stderr", &FLAGS_logtostderr);
  AddAndRegisterDefaultOption("log_level", &FLAGS_v);
}

void OptionManager::Reset() {
  desc_.reset(new boost::program_options::options_description());
  
  ResetOptions();

  options_bool_.clear();
  options_int_.clear();
  options_double_.clear();
  options_string_.clear();

  added_reader_options_ = false;
  added_vanishing_point_options_ = false;
  added_feature_matching_options_ = false;
}

void OptionManager::ResetOptions(){
  *reader = ReaderOptions();
  *vanishing_point = VanishingPointOptions();
  *feature_matching = FeatureMatchingOptions();
}

bool OptionManager::Check() {
  bool success = true;
  
  if (reader) success = success && reader->Check();

  return success;
}

void OptionManager::Parse(const int argc, char** argv) {
  config::variables_map vmap;

  try {
    config::store(config::parse_command_line(argc, argv, *desc_), vmap);

    if (vmap.count("help")) {
      //std::cout << StringPrintf("%s (%s)", GetVersionInfo().c_str(),
      //                          GetBuildInfo().c_str())
      //          << std::endl
      //          << std::endl;
      std::cout
          << "Options can either be specified via command-line or by defining"
          << std::endl
          << "them in a .ini project file passed to `--project_path`."
          << std::endl
          << std::endl;
      std::cout << *desc_ << std::endl;
      exit(EXIT_SUCCESS);
    }

    //if (vmap.count("project_path")) {
    //  *project_path = vmap["project_path"].as<std::string>();
    //  if (!Read(*project_path)) {
    //    exit(EXIT_FAILURE);
    //  }
    //} else {
    //  vmap.notify();
    //}
    vmap.notify();
  } catch (std::exception& exc) {
    std::cerr << "ERROR: Failed to parse options - " << exc.what() << "."
              << std::endl;
    exit(EXIT_FAILURE);
  } catch (...) {
    std::cerr << "ERROR: Failed to parse options for unknown reason."
              << std::endl;
    exit(EXIT_FAILURE);
  }

  if (!Check()) {
    std::cerr << "ERROR: Invalid options provided." << std::endl;
    exit(EXIT_FAILURE);
  }
}

bool OptionManager::Read(const std::string& path) {
  config::variables_map vmap;

  if (!colmap::ExistsFile(path)) {
    std::cout << "ERROR: Configuration file does not exist." << std::endl;
    return false;
  }

  try {
    std::ifstream file(path);
    CHECK(file.is_open()) << path;
    config::store(config::parse_config_file(file, *desc_), vmap);
    vmap.notify();
  } catch (std::exception& e) {
    std::cout << "ERROR: Failed to parse options " << e.what() << "."
              << std::endl;
    return false;
  } catch (...) {
    std::cout << "ERROR: Failed to parse options for unknown reason."
              << std::endl;
    return false;
  }

  return Check();
}

void OptionManager::Write(const std::string& path) const {
  boost::property_tree::ptree pt;

  // First, put all options without a section and then those with a section.
  // This is necessary as otherwise older Boost versions will write the
  // options without a section in between other sections and therefore
  // the errors will be assigned to the wrong section if read later.

  for (const auto& option : options_bool_) {
    if (!colmap::StringContains(option.first, ".")) {
      pt.put(option.first, *option.second);
    }
  }

  for (const auto& option : options_int_) {
    if (!colmap::StringContains(option.first, ".")) {
      pt.put(option.first, *option.second);
    }
  }

  for (const auto& option : options_double_) {
    if (!colmap::StringContains(option.first, ".")) {
      pt.put(option.first, *option.second);
    }
  }

  for (const auto& option : options_string_) {
    if (!colmap::StringContains(option.first, ".")) {
      pt.put(option.first, *option.second);
    }
  }

  for (const auto& option : options_bool_) {
    if (colmap::StringContains(option.first, ".")) {
      pt.put(option.first, *option.second);
    }
  }

  for (const auto& option : options_int_) {
    if (colmap::StringContains(option.first, ".")) {
      pt.put(option.first, *option.second);
    }
  }

  for (const auto& option : options_double_) {
    if (colmap::StringContains(option.first, ".")) {
      pt.put(option.first, *option.second);
    }
  }

  for (const auto& option : options_string_) {
    if (colmap::StringContains(option.first, ".")) {
      pt.put(option.first, *option.second);
    }
  }

  boost::property_tree::write_ini(path, pt);
}

void OptionManager::AddReaderOptions(){
  if (added_reader_options_){
    return;
  }
  added_reader_options_ = true;

  AddAndRegisterDefaultOption("Reader.scene_path",
                              &reader->scene_path);
  AddAndRegisterDefaultOption("Reader.image_path",
                              &reader->image_path);
  AddAndRegisterDefaultOption("Reader.camera_path",
                              &reader->camera_path);
  AddAndRegisterDefaultOption("Reader.vanishing_point_path",
                              &reader->vanishing_point_path);
  AddAndRegisterDefaultOption("Reader.line_segment_path",
                              &reader->line_segment_path);
  AddAndRegisterDefaultOption("Reader.plane_path",
                              &reader->plane_path);
  AddAndRegisterDefaultOption("Reader.feature_path",
                              &reader->feature_path);
  AddAndRegisterDefaultOption("Reader.match_path",
                              &reader->match_path);
  AddAndRegisterDefaultOption("Reader.output_path",
                              &reader->output_path);
  AddAndRegisterDefaultOption("Reader.config_path",
                              &reader->config_path);
  AddAndRegisterDefaultOption("Reader.image_id",
                              &reader->image_id);
  AddAndRegisterDefaultOption("Reader.image_id1",
                              &reader->image_id1);
  AddAndRegisterDefaultOption("Reader.image_id2",
                              &reader->image_id2);
  AddAndRegisterDefaultOption("Reader.undistort",
                              &reader->undistort);
  AddAndRegisterDefaultOption("Reader.undistortion_max_size",
                              &reader->undistortion_max_size);
  AddAndRegisterDefaultOption("Reader.line_segment_type",
                              &reader->line_segment_type);
  AddAndRegisterDefaultOption("Reader.exec_mode",
                              &reader->exec_mode);
}

void OptionManager::AddVanishingPointOptions(){
  if (added_vanishing_point_options_){
    return;
  }
  added_vanishing_point_options_ = true;

  AddAndRegisterDefaultOption("VanishingPoint.min_num_inliers",
                              &vanishing_point->min_num_inliers);
  AddAndRegisterDefaultOption("VanishingPoint.min_remaining_lines_ratio",
                              &vanishing_point->min_remaining_lines_ratio);
  AddAndRegisterDefaultOption("VanishingPoint.min_line_segment_size",
                              &vanishing_point->min_line_segment_size);
  AddAndRegisterDefaultOption("VanishingPoint.min_line_segment_score",
                              &vanishing_point->min_line_segment_score);
  AddAndRegisterDefaultOption("VanishingPoint.line_orientation_tolerance",
                              &vanishing_point->line_orientation_tolerance);
  AddAndRegisterDefaultOption("VanishingPoint.max_vanishing_points_num",
                              &vanishing_point->max_vanishing_points_num);
  AddAndRegisterDefaultOption("VanishingPoint.max_error",
                              &vanishing_point->max_error);
  AddAndRegisterDefaultOption("VanishingPoint.max_error2",
                              &vanishing_point->max_error2);
  AddAndRegisterDefaultOption("VanishingPoint.confidence",
                              &vanishing_point->confidence);
  AddAndRegisterDefaultOption("VanishingPoint.min_num_trials",
                              &vanishing_point->min_num_trials);
  AddAndRegisterDefaultOption("VanishingPoint.max_num_trials",
                              &vanishing_point->max_num_trials);
  AddAndRegisterDefaultOption("VanishingPoint.min_inlier_ratio",
                              &vanishing_point->min_inlier_ratio);
  AddAndRegisterDefaultOption("VanishingPoint.undistort",
                              &vanishing_point->undistort);
}

void OptionManager::AddFeatureMatchingOptions(){
  if (added_feature_matching_options_){
    return;
  }
  added_feature_matching_options_ = true;

  AddAndRegisterDefaultOption("FeatureMatching.use_gpu", 
                              &feature_matching->use_gpu);
  AddAndRegisterDefaultOption("FeatureMatching.max_ratio",
                              &feature_matching->max_ratio);
  AddAndRegisterDefaultOption("FeatureMatching.max_distance",
                              &feature_matching->max_distance);
  AddAndRegisterDefaultOption("FeatureMatching.ratio_test_radius",
                              &feature_matching->ratio_test_radius);
  AddAndRegisterDefaultOption("FeatureMatching.cross_check",
                              &feature_matching->cross_check);
  AddAndRegisterDefaultOption("FeatureMatching.max_error",
                              &feature_matching->max_error);
  AddAndRegisterDefaultOption("FeatureMatching.confidence",
                              &feature_matching->confidence);
  AddAndRegisterDefaultOption("FeatureMatching.min_num_trials",
                              &feature_matching->min_num_trials);
  AddAndRegisterDefaultOption("FeatureMatching.max_num_trials",
                              &feature_matching->max_num_trials);
  AddAndRegisterDefaultOption("FeatureMatching.min_inlier_ratio",
                              &feature_matching->min_inlier_ratio);
  AddAndRegisterDefaultOption("FeatureMatching.min_num_inliers",
                              &feature_matching->min_num_inliers);
}


} // namespace goma
