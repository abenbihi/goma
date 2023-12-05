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
//  - New line segment struct
//  - Update the prototype of colmap::ClassifyLineSegmentOrientations to
//    accomdate the new struct
//  - Preprocessing functionalities on the line segments

#include <glog/logging.h>

#include "goma/scene/line_segment.h"

namespace goma {

void ClassifyLineSegmentOrientations(
  const float tolerance,
  std::vector<LineSegment>& line_segments) {
  CHECK_LE(tolerance, 0.5);
  for(size_t i=0; i<line_segments.size(); i++){
    line_segments[i].ClassifyOrientation(tolerance);
  }
}

void ClassifyLineSegmentOrientations(const float tolerance,
    LineSegments& line_segments){
  CHECK_LE(tolerance, 0.5);
  for (auto& line_segment : line_segments){
    line_segment.second.ClassifyOrientation(tolerance);
  }
}

void FilterLineSegments(const float score,
    const float min_line_segment_size, 
    LineSegments& line_segments){
  std::vector<int> filtered_line_segment_ids;

  for (const auto line_segment : line_segments){
    // Remove line segments with low confidence
    if (line_segment.second.score < score){
      filtered_line_segment_ids.push_back(line_segment.first);
      continue;
    }

    // Remove line segments with small length
    double length = (line_segment.second.start - line_segment.second.end).norm();
    if (length < min_line_segment_size){
      filtered_line_segment_ids.push_back(line_segment.first);
      continue;
    }
  }

  // Only de-register after iterating over line_segment_ids to avoid
  // simultaneous iteration and modification of the vector.
  for (size_t i=0; i<filtered_line_segment_ids.size(); i++){
    int line_segment_id = filtered_line_segment_ids[i];
    line_segments.erase(line_segment_id);
  }
}

} // namespace goma
