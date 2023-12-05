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

#ifndef GOMA_SCENE_SCENE_H
#define GOMA_SCENE_SCENE_H

#include "base/camera.h" // for the colmap camera model

#include "goma/scene/image.h"
#include "goma/util/reader.h"

namespace goma {

struct Scene {
  Scene() { }
  
  Scene(const ReaderOptions& options);

  ~Scene() { }

  // Set the path to read data from.
  void SetPaths(const ReaderOptions& options){
    scene_path = options.scene_path;
    image_path = options.scene_path + "/images.txt";
    camera_path = options.scene_path + "/cameras.txt";
  }

  void ReadCamerasText(const std::string& path);
  void ReadImagesText(const std::string& path);

  EIGEN_STL_UMAP(colmap::camera_t, class colmap::Camera) cameras;
  EIGEN_STL_UMAP(int, class Image) images;

  std::string scene_path;
  std::string image_path;
  std::string camera_path;
  std::vector<int> image_ids;
};

} // namespace goma

#endif // GOMA_SCENE_SCENE_H
