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
//  - Simpler struct to store the images and cameras information.

#include <fstream>

#include "colmap/base/pose.h"
#include "colmap/util/string.h"
#include "colmap/util/misc.h"

#include "goma/scene/scene.h"
#include "goma/util/loader.h"

namespace goma {

Scene::Scene(const ReaderOptions& options){
  scene_path = options.scene_path;
  image_path = options.scene_path + "/images.txt";
  camera_path = options.scene_path + "/cameras.txt";

  ReadCamerasText(camera_path);
  ReadImagesText(image_path);
}

void Scene::ReadCamerasText(const std::string& path){
  cameras.clear();

  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  while (std::getline(file, line)) {
    colmap::StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    class colmap::Camera camera;

    // ID
    std::getline(line_stream, item, ' ');
    camera.SetCameraId(std::stoul(item));

    // MODEL
    std::getline(line_stream, item, ' ');
    camera.SetModelIdFromName(item);

    // WIDTH
    std::getline(line_stream, item, ' ');
    camera.SetWidth(std::stoll(item));

    // HEIGHT
    std::getline(line_stream, item, ' ');
    camera.SetHeight(std::stoll(item));

    // PARAMS
    camera.Params().clear();
    while (!line_stream.eof()) {
      std::getline(line_stream, item, ' ');
      camera.Params().push_back(std::stold(item));
    }

    CHECK(camera.VerifyParams());

    cameras.emplace(camera.CameraId(), camera);
  }
}

void Scene::ReadImagesText(const std::string& path){
  images.clear();

  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  while (std::getline(file, line)) {
    colmap::StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream1(line);

    // ID
    std::getline(line_stream1, item, ' ');
    const colmap::image_t image_id = std::stoul(item);

    class Image image;
    image.image_id = image_id;

    image_ids.push_back(image_id);

    // QVEC (qw, qx, qy, qz)
    std::getline(line_stream1, item, ' ');
    image.qvec(0) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.qvec(1) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.qvec(2) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.qvec(3) = std::stold(item);

    image.qvec = colmap::NormalizeQuaternion(image.qvec);

    // TVEC
    std::getline(line_stream1, item, ' ');
    image.tvec(0) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.tvec(1) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.tvec(2) = std::stold(item);

    // CAMERA_ID
    std::getline(line_stream1, item, ' ');
    image.camera_id = std::stoul(item);

    // NAME
    std::getline(line_stream1, item, ' ');
    image.image_name = item;

    image.camera = cameras[image.camera_id];

    images.emplace(image.image_id, image);
  }
}

} // namespace goma
