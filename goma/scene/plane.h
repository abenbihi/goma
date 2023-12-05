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

#ifndef GOMA_BASE_PLANE_H
#define GOMA_BASE_PLANE_H

namespace goma {

struct Plane {
  int id;
  int xmin;
  int xmax;
  int ymin;
  int ymax;
  int horizontal_vanishing_point_id;
  int vertical_vanishing_point_id;
  Eigen::Vector2d horizontal_vanishing_point;
  Eigen::Vector2d vertical_vanishing_point;
  Eigen::Vector3d h_horizontal_vanishing_point;
  Eigen::Vector3d h_vertical_vanishing_point;

  std::vector<Eigen::Vector2d> corners;

  std::vector<LineSegment> edges;
  //bool are_edges_set;

  //Triangle triangle0_;
  //Triangle triangle1_;
  //bool are_triangles_set_;

  Plane()
  : id(-1),
    xmin(-1),
    xmax(-1),
    ymin(-1),
    ymax(-1),
    horizontal_vanishing_point_id(-1),
    vertical_vanishing_point_id(-1),
    horizontal_vanishing_point(Eigen::Vector2d(0,0)),
    vertical_vanishing_point(Eigen::Vector2d(0,0)),
    h_horizontal_vanishing_point(Eigen::Vector3d(0,0,0)),
    h_vertical_vanishing_point(Eigen::Vector3d(0,0,0)) { }
    //are_edges_set_(false),
    //are_triangles_set_(false) { }

  Plane(const int id, const int xmin, const int xmax, 
      const Eigen::Vector2d& horizontal_vanishing_point,
      const Eigen::Vector2d& vertical_vanishing_point)
  : id(id),
    xmin(xmin),
    xmax(xmax),
    horizontal_vanishing_point(horizontal_vanishing_point),
    vertical_vanishing_point(vertical_vanishing_point) { }
    //are_edges_set_(false),
    //are_triangles_set_(false) { }

  
  ~Plane() {}

  void Expand(int delta, int image_width, int image_height);

  void SetCorners(int image_width, int image_height);

  void SetEdges();

  void SetTriangles();

  bool Hold(const Eigen::Vector2d& point) const;

  bool Hold(const LineSegment& line_segment) const;

  bool Hold(const LineSegment& line_segment, LineSegment& line_segment_out) const;

  //bool Hold(const Box& box) const;

  bool HoldBand(const Eigen::Vector2d& point) const;
  
  bool HoldBand(const LineSegment& line_segment) const;
  
  //bool HoldBand(const Box& box) const;
};


}

#endif // GOMA_BASE_PLANE_H
