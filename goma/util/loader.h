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

#ifndef GOMA_UTIL_LOADER_H
#define GOMA_UTIL_LOADER_H

#include <string>

#include "goma/scene/line_segment.h"
#include "goma/scene/plane.h"
#include "goma/scene/vanishing_point.h"

namespace goma {

void WriteLineSegments(const std::string& path,
  const std::vector<LineSegment>& line_segments);

LineSegments ReadLineSegments(const std::string& path);

std::vector<LineSegment> ReadLineSegmentsVector(const std::string& path);

void WriteVanishingPoints(const std::string& path,
    const std::vector<VanishingPoint>& vanishing_points);

std::vector<VanishingPoint> ReadVanishingPoints(const std::string& path);

std::vector<Plane> ReadPlanes(const std::string& path);

std::vector<Eigen::Vector2d> ReadPoints(const std::string& path);

void WriteHomography(const std::string& path, const Eigen::Matrix3d& H);

std::string GenMatchesPath(const std::string& output_dir,
    const std::string& image_name1,
    const std::string& image_name2);

} // namespace goma

#endif // GOMA_UTIL_LOADER_H
