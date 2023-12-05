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

// Colmap includes
#include "base/camera.h"
#include "util/misc.h"

#include "goma/feature/lsd.h"
#include "goma/feature/vanishing_point.h"
#include "goma/scene/scene.h"
#include "goma/util/loader.h"
#include "goma/util/reader.h"
#include "goma/util/option_manager.h"

#include "examples/image.h"

namespace goma {

int RunLineSegmentDetector(int argc, char* argv[]){
  // Parse options
  OptionManager options;
  options.AddReaderOptions();
  options.AddVanishingPointOptions();
  options.Parse(argc, argv);

  ReaderOptions reader = *options.reader;
  options.Write(reader.config_path);

  VanishingPointOptions vanishing_point_options = *options.vanishing_point;

  Scene scene(reader);
  for (size_t i=0; i<scene.image_ids.size(); i++){
    if ((i%100)==0){
      printf("%ld / %ld\n", i, scene.image_ids.size());
    }

    int image_id = scene.image_ids[i];
    std::string image_name = scene.images[image_id].image_name;
    int camera_id = scene.images[image_id].camera_id;

    std::string image_path = colmap::JoinPaths(reader.image_path, image_name);
    colmap::Camera distorted_camera = scene.cameras[camera_id];

    // Detect line segments
    std::vector<LineSegment> line_segments = DetectColmapLineSegments(
        image_path,
        distorted_camera,
        vanishing_point_options.min_line_segment_size,
        reader.undistortion_max_size);

    // Save line segments to file
    std::string line_segment_path = colmap::JoinPaths(reader.line_segment_path, image_name) + ".txt";
    WriteLineSegments(line_segment_path, line_segments);
  }

  return 0;
}

int RunVanishingPointDetector(int argc, char* argv[]){
  // Parse options
  OptionManager options;
  options.AddReaderOptions();
  options.AddVanishingPointOptions();
  options.Parse(argc, argv);

  ReaderOptions reader = *options.reader;
  options.Write(reader.config_path);

  VanishingPointOptions vanishing_point_options = *options.vanishing_point;

  Scene scene(reader);

  for (size_t i=0; i<scene.image_ids.size(); i++){
    if ((i%100)==0){
      printf("%ld / %ld\n", i, scene.image_ids.size());
    }

    int image_id = scene.image_ids[i];
    std::string image_name = scene.images[image_id].image_name;
    int camera_id = scene.images[image_id].camera_id;
    colmap::Camera distorted_camera = scene.cameras[camera_id];

    // Read line segments.
    std::string line_segment_path = colmap::JoinPaths(reader.line_segment_path, image_name) + ".txt";
    LineSegments line_segments = ReadLineSegments(line_segment_path);
    
    // Preprocess line segments
    PreprocessLineSegments(vanishing_point_options, line_segments);

    // Estimate vanishing points.
    std::vector<VanishingPoint> vanishing_points;
    DetectVanishingPoints(vanishing_point_options, line_segments,
        vanishing_points);

    // Write vanishing points to file.
    std::string vanishing_point_path = colmap::JoinPaths(
        reader.vanishing_point_path, image_name) + ".txt";
    WriteVanishingPoints(vanishing_point_path, vanishing_points);
  }

  return 0;
}

} // namespace goma
