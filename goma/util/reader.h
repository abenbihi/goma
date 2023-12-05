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

#ifndef GOMA_UTIL_READER_H
#define GOMA_UTIL_READER_H

#include <string.h>

namespace goma {

  struct ReaderOptions {
    std::string scene_path = "data/sample_aachen/scene/";
    std::string image_path = "data/sample_aachen/images/";
    std::string camera_path = ""; 

    std::string line_segment_path = "data/sample_aachen/line_segments/";
    std::string plane_path = "data/sample_aachen/planes/";
    std::string vanishing_point_path = "data/sample_aachen/vanishing_points/"; 
    std::string feature_path = "data/sample_aachen/features/";
    
    std::string match_path = "data/sample_aachen/features/";

    std::string output_path = "data/sample_aachen/outputs/";

    std::string config_path = "./data/sample_aachen/config.txt";

    int image_id = 5304;
    int image_id1 = 5304;
    int image_id2 = 3254;

    int undistort = 0;

    int undistortion_max_size = 1600;

    std::string line_segment_type = "lsd";

    // {0:pair, 1:list}
    int exec_mode = 0;

    bool Check(){
      return true;
    }
  };

} // namespace goma

#endif // GOMA_UTIL_READER_H
