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

#include <fstream>
#include <glog/logging.h>

#include "util/misc.h"

#include "goma/util/loader.h"

namespace goma {

void WriteLineSegments(const std::string& path,
    const std::vector<LineSegment>& line_segments){
  
  std::ofstream file(path, std::ios::trunc);
  CHECK(file.is_open()) << path;

  // Ensure that we don't loose any precision by storing in text.
  file.precision(17);

  std::ostringstream line;
  std::string line_string;

  // Write header.
  //  LINE_0:   NUM_LINE_SEGMENTS
  line << line_segments.size();
  line_string = line.str();
  file << line_string << std::endl;
  line.str("");
  line.clear();

  for (size_t i=0; i<line_segments.size(); i++){
    line << line_segments[i].start.x() << " ";
    line << line_segments[i].start.y() << " ";
    line << line_segments[i].end.x() << " ";
    line << line_segments[i].end.y() << " ";
    line << line_segments[i].score;

    // Format data.
    line_string = line.str();
    file << line_string << std::endl;
    line.str("");
    line.clear();
  }
}

LineSegments ReadLineSegments(const std::string& path){
  LineSegments line_segments;

  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  std::getline(file, line);
  std::stringstream header_line_stream(line);

  std::getline(header_line_stream >> std::ws, item, ' ');
  const int num_lines = std::stoul(item);

  for (size_t i=0; i<(size_t)num_lines; ++i){
    std::getline(file, line);
    std::stringstream line_stream(line);

    std::getline(line_stream >> std::ws, item, ' ');
    const float x0 = std::stold(item);

    std::getline(line_stream >> std::ws, item, ' ');
    const float y0 = std::stold(item);

    std::getline(line_stream >> std::ws, item, ' ');
    const float x1 = std::stold(item);

    std::getline(line_stream >> std::ws, item, ' ');
    const float y1 = std::stold(item);

    std::getline(line_stream >> std::ws, item, ' ');
    const float score = std::stold(item);

    class LineSegment seg;
    seg.start = Eigen::Vector2d(x0,y0);
    seg.end = Eigen::Vector2d(x1,y1);
    seg.score = score;
    seg.line_segment_id = i;
    seg.registered = true;

    line_segments.emplace(seg.line_segment_id, seg);
  }
  file.close();

  return line_segments;
}

std::vector<LineSegment> ReadLineSegmentsVector(const std::string& path){
  std::vector<LineSegment> line_segments;

  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  std::getline(file, line);
  std::stringstream header_line_stream(line);

  std::getline(header_line_stream >> std::ws, item, ' ');
  const int num_lines = std::stoul(item);

  for (size_t i=0; i<(size_t)num_lines; ++i){
    std::getline(file, line);
    std::stringstream line_stream(line);

    std::getline(line_stream >> std::ws, item, ' ');
    const float x0 = std::stold(item);

    std::getline(line_stream >> std::ws, item, ' ');
    const float y0 = std::stold(item);

    std::getline(line_stream >> std::ws, item, ' ');
    const float x1 = std::stold(item);

    std::getline(line_stream >> std::ws, item, ' ');
    const float y1 = std::stold(item);

    std::getline(line_stream >> std::ws, item, ' ');
    const float score = std::stold(item);

    class LineSegment seg;
    seg.start = Eigen::Vector2d(x0,y0);
    seg.end = Eigen::Vector2d(x1,y1);
    seg.score = score;
    seg.line_segment_id = i;
    seg.registered = true;

    line_segments.push_back(seg);
  }
  file.close();

  return line_segments;
}

void WriteVanishingPoints(const std::string& path,
    const std::vector<VanishingPoint>& vanishing_points){
  std::ofstream file(path, std::ios::trunc);
  CHECK(file.is_open()) << path;

  // Ensure that we don't loose any precision by storing in text.
  file.precision(17);

  std::ostringstream line;
  std::string line_string;

  // Write header.
  // VANISHING_POINTS_NUM
  line << vanishing_points.size();
  line_string = line.str();
  file << line_string << std::endl;
  line.str("");
  line.clear();

  // Write data with one line per file line and the following format:
  // VANISHING_POINT_ID HX HY HZ ORIENTATION LINE_SEGMENT_NUM LINE_SEGMENT_ID0 ...
  for (size_t i=0; i<vanishing_points.size(); i++){
    line << vanishing_points[i].id << " ";
    
    //line << vanishing_points[i].vp.x() << " ";
    //line << vanishing_points[i].vp.y() << " ";
    
    line << vanishing_points[i].hvp.x() << " ";
    line << vanishing_points[i].hvp.y() << " ";
    line << vanishing_points[i].hvp.z() << " ";

    line << vanishing_points[i].orientation << " ";
    line << vanishing_points[i].inlier_line_segment_ids.size() << " ";
    for (size_t j=0; j<vanishing_points[i].inlier_line_segment_ids.size(); j++){
      line << vanishing_points[i].inlier_line_segment_ids[j] << " ";
    }

    // Format data.
    line_string = line.str();
    line_string = line_string.substr(0, line_string.size() - 1);
    file << line_string << std::endl;
    line.str("");
    line.clear();
  }
}

std::vector<VanishingPoint> ReadVanishingPoints(const std::string& path){
  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  std::getline(file, line);
  std::stringstream header_line_stream(line);

  std::getline(header_line_stream >> std::ws, item, ' ');
  const int num_vanishing_points = std::stoul(item);

  std::vector<VanishingPoint> vanishing_points;
  for (size_t i=0; i<(size_t)num_vanishing_points; ++i){
    std::getline(file, line);
    std::stringstream feature_line_stream(line);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    int vanishing_point_id = std::stoul(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const double x = std::stod(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const double y = std::stod(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const double z = std::stod(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const VanishingPoint::Orientation orientation = 
      (VanishingPoint::Orientation) std::stoul(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    size_t line_segment_num = std::stoul(item);

    std::vector<int> inlier_line_segment_ids;
    for (size_t j=0; j<line_segment_num; j++){
      std::getline(feature_line_stream >> std::ws, item, ' ');
      int line_segment_id = (int) std::stoul(item);
      inlier_line_segment_ids.push_back(line_segment_id);
    }

    VanishingPoint vanishing_point;
    vanishing_point.id = vanishing_point_id;
    vanishing_point.hvp = Eigen::Vector3d(x,y,z);
    vanishing_point.vp = vanishing_point.hvp.hnormalized();
    vanishing_point.orientation = orientation;
    vanishing_point.inlier_line_segment_ids = inlier_line_segment_ids;
    vanishing_point.num_inliers = inlier_line_segment_ids.size();
    vanishing_points.push_back(vanishing_point);
  }
  
  return vanishing_points;
}

std::vector<Eigen::Vector2d> ReadPoints(const std::string& path){
  std::vector<Eigen::Vector2d> keypoints;
  std::ifstream file(path.c_str());
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  std::getline(file, line);
  std::stringstream header_line_stream(line);

  std::getline(header_line_stream >> std::ws, item, ' ');
  const colmap::point2D_t num_features = std::stoul(item);

  keypoints.resize(num_features);

  for (size_t i = 0; i < num_features; ++i) {
    std::getline(file, line);
    std::stringstream feature_line_stream(line);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const float x = std::stold(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const float y = std::stold(item);

    keypoints[i] = Eigen::Vector2d(x, y);
  }

  file.close();
  
  return keypoints;
}

std::vector<Plane> ReadPlanes(const std::string& path){
  std::vector<Plane> planes;

  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  std::getline(file, line);
  std::stringstream header_line_stream(line);

  std::getline(header_line_stream >> std::ws, item, ' ');
  const int num_planes = std::stoul(item);

  for (size_t i=0; i<(size_t)num_planes; ++i){
    std::getline(file, line);
    std::stringstream feature_line_stream(line);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    int plane_id = std::stoul(item);

    // Read plane corners.
    std::vector<Eigen::Vector2d> corners;
    for (size_t j=0; j<4; j++){
      std::getline(feature_line_stream >> std::ws, item, ' ');
      const float x = std::stold(item);

      std::getline(feature_line_stream >> std::ws, item, ' ');
      const float y = std::stold(item);

      Eigen::Vector2d c(x,y);
      corners.push_back(c);
    }
    double xmin = std::min(corners[0].x(), corners[1].x());
    double xmax = std::max(corners[2].x(), corners[3].x());

    // Read horizontal vanishing point coordinates.
    std::getline(feature_line_stream >> std::ws, item, ' ');
    const int horizontal_vanishing_point_id = std::stoul(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const float hx = std::stold(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const float hy = std::stold(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const float hz = std::stold(item);

    //Eigen::Vector2d horizontal_vanishing_point(hx,hy);
    Eigen::Vector3d h_horizontal_vanishing_point(hx,hy,hz);
    Eigen::Vector2d horizontal_vanishing_point = 
      h_horizontal_vanishing_point.hnormalized();

    // Read vertical vanishing point coordinates.
    std::getline(feature_line_stream >> std::ws, item, ' ');
    const int vertical_vanishing_point_id = std::stoul(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const float vx = std::stold(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const float vy = std::stold(item);

    std::getline(feature_line_stream >> std::ws, item, ' ');
    const float vz = std::stold(item);

    Eigen::Vector3d h_vertical_vanishing_point(vx,vy,vz);
    Eigen::Vector2d vertical_vanishing_point = 
      h_vertical_vanishing_point.hnormalized();

    Plane plane;
    plane.id = plane_id;
    plane.xmin = xmin;
    plane.xmax = xmax;
    plane.ymin = 0;
    plane.ymax = corners[1].y();
    plane.horizontal_vanishing_point_id = horizontal_vanishing_point_id;
    plane.h_horizontal_vanishing_point = h_horizontal_vanishing_point;
    plane.horizontal_vanishing_point = horizontal_vanishing_point;
    plane.vertical_vanishing_point_id = vertical_vanishing_point_id;
    plane.h_vertical_vanishing_point = h_vertical_vanishing_point;
    plane.vertical_vanishing_point = vertical_vanishing_point;
    plane.corners = corners;

    planes.push_back(plane);
  }

  return planes;
}

void WriteHomography(const std::string& path, const Eigen::Matrix3d& H){
  std::ofstream file(path, std::ios::trunc);
  CHECK(file.is_open()) << path;

  // Ensure that we don't loose any precision by storing in text.
  file.precision(17);

  std::ostringstream line;
  std::string line_string;
  for (size_t i=0; i<(size_t)H.rows(); i++){
    for (size_t j=0; j<(size_t)H.cols(); j++){
      line << H(i,j) << " ";
    }
  }
  line_string = line.str();
  line_string = line_string.substr(0, line_string.size() - 1);
  file << line_string << std::endl;
  line.str("");
  line.clear();
}

std::string GenMatchesPath(const std::string& output_dir,
    const std::string& image_name1,
    const std::string& image_name2){
  std::string base_path1;
  std::string base_path2;
  std::string extension;
  colmap::SplitFileExtension(image_name1, &base_path1, &extension);
  colmap::SplitFileExtension(image_name2, &base_path2, &extension);

  base_path1 = colmap::StringReplace(base_path1, "/", "-");
  base_path2 = colmap::StringReplace(base_path2, "/", "-");

  std::string path = colmap::JoinPaths(output_dir, 
      base_path1 + "_" + base_path2) + ".txt";
  //std::cout << "output_path: " << path << std::endl;

  return path;
}

} // namespace goma
