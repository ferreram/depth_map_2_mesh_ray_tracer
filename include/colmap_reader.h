#pragma once

#include <iostream>
#include <string>

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>

class ColmapReader {

public:
  void Read(const std::string& path);

  void DisplayCameraCalib() const;

  void DisplayRegisteredImages() const;

  int m_img_width = -1;
  int m_img_height = -1;

  double m_fx = -1.0;
  double m_fy = -1.0;
  double m_cx = -1.0;
  double m_cy = -1.0;

  std::unordered_map<std::string, Sophus::SE3d> m_map_img_names_T_world_cam;

private:
  void ReadText(const std::string& path);
  void ReadBinary(const std::string& path);

  void ReadCamerasText(const std::string& path);
  void ReadImagesText(const std::string& path);
  // void ReadPoints3DText(const std::string& path);

  void ReadCamerasBinary(const std::string& path);
  void ReadImagesBinary(const std::string& path);
  // void ReadPoints3DBinary(const std::string& path);

};
