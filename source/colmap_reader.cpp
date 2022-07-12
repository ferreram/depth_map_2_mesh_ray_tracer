#include "colmap_reader.h"

#include "files_utils.h"

void ColmapReader::Read(const std::string& path) {
  if (ExistsFile(JoinPaths(path, "cameras.bin")) &&
      ExistsFile(JoinPaths(path, "images.bin")) &&
      ExistsFile(JoinPaths(path, "points3D.bin"))) {
    ReadBinary(path);
  } else if (ExistsFile(JoinPaths(path, "cameras.txt")) &&
            ExistsFile(JoinPaths(path, "images.txt")) &&
            ExistsFile(JoinPaths(path, "points3D.txt"))) {
    ReadText(path);
  } else {
    std::cerr << "cameras, images, points3D files do not exist at " << path;
  }
}

void ColmapReader::ReadText(const std::string& path) {
  ReadCamerasText(JoinPaths(path, "cameras.txt"));
  ReadImagesText(JoinPaths(path, "images.txt"));
  // ReadPoints3DText(JoinPaths(path, "points3D.txt"));
}

void ColmapReader::ReadBinary(const std::string& path) {
  ReadCamerasBinary(JoinPaths(path, "cameras.bin"));
  ReadImagesBinary(JoinPaths(path, "images.bin"));
  // ReadPoints3DBinary(JoinPaths(path, "points3D.bin"));
}


void ColmapReader::ReadCamerasText(const std::string& path) {

  std::ifstream file(path);
  // CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  while (std::getline(file, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    if (m_img_width > 0)
    {
      std::cerr << "ERROR! ONLY SINGLE CAMERA CASE IS SUPPORTED RIGHT NOW!\n\n";
      std::cerr << "NUMBER OF CAMERAS IN INPUT MODEL IS AT LEAST TWO!\n\n";
      exit(-1);
    }

    std::stringstream line_stream(line);

    // class Camera camera;

    // ID
    std::getline(line_stream, item, ' ');
    // camera.SetCameraId(std::stoul(item));

    // MODEL
    std::getline(line_stream, item, ' ');
    // camera.SetModelIdFromName(item);
    if (item != "PINHOLE")
    {
      std::cerr << "ERROR! EXPECTED CAMERA MODEL SHOULD BE : PINHOLE (i.e. 1 in Colmap)!\n\n";
      std::cerr << "INPUT MODEL IS : " << item << "\n\n";
      exit(-1);
    }

    // WIDTH
    std::getline(line_stream, item, ' ');
    // camera.SetWidth(std::stoll(item));
    m_img_width = static_cast<int>(std::stoll(item));

    // HEIGHT
    std::getline(line_stream, item, ' ');
    // camera.SetHeight(std::stoll(item));
    m_img_height = static_cast<int>(std::stoll(item));

    // Focal X
    std::getline(line_stream, item, ' ');
    m_fx = static_cast<double>(std::stold(item));

    // Focal Y
    std::getline(line_stream, item, ' ');
    m_fy = static_cast<double>(std::stold(item));

    // Principal Point X
    std::getline(line_stream, item, ' ');
    m_cx = static_cast<double>(std::stold(item));

    // Principal Point Y
    std::getline(line_stream, item, ' ');
    m_cy = static_cast<double>(std::stold(item));
  }
}

void ColmapReader::ReadImagesText(const std::string& path) {
  // images_.clear();

  std::ifstream file(path);
  // CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  while (std::getline(file, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream1(line);

    // ID
    std::getline(line_stream1, item, ' ');
    // const uint32_t image_id = static_cast<uint32_t>(std::stoul(item));

    // class Image image;
    // image.SetImageId(image_id);

    // image.SetRegistered(true);
    // reg_image_ids_.push_back(image_id);

    // QVEC (qw, qx, qy, qz)
    std::getline(line_stream1, item, ' ');
    // image.Qvec(0) = std::stold(item);
    const double qw = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Qvec(1) = std::stold(item);
    const double qx = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Qvec(2) = std::stold(item);
    const double qy = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Qvec(3) = std::stold(item);
    const double qz = std::stold(item);

    Eigen::Quaterniond qcw(qw, qx, qy, qz);
    qcw.normalize();

    // image.NormalizeQvec();

    // TVEC
    std::getline(line_stream1, item, ' ');
    // image.Tvec(0) = std::stold(item);
    const double tx = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Tvec(1) = std::stold(item);
    const double ty = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Tvec(2) = std::stold(item);
    const double tz = std::stold(item);

    const Eigen::Vector3d tcw(tx, ty, tz);

    // Transformation from world frame to cam frame
    const Sophus::SE3d T_cam_world(qcw, tcw);

    // CAMERA_ID
    std::getline(line_stream1, item, ' ');
    // image.SetCameraId(std::stoul(item));

    // NAME
    std::getline(line_stream1, item, ' ');
    // image.SetName(item);

    // We store the inverse transformation
    m_map_img_names_T_world_cam.emplace(item, T_cam_world.inverse());

    // // POINTS2D
    if (!std::getline(file, line)) {
      break;
    }

    // StringTrim(&line);
    // std::stringstream line_stream2(line);

    // std::vector<Eigen::Vector2d> points2D;
    // std::vector<point3D_t> point3D_ids;

    // if (!line.empty()) {
    //   while (!line_stream2.eof()) {
    //     Eigen::Vector2d point;

    //     std::getline(line_stream2, item, ' ');
    //     point.x() = std::stold(item);

    //     std::getline(line_stream2, item, ' ');
    //     point.y() = std::stold(item);

    //     points2D.push_back(point);

    //     std::getline(line_stream2, item, ' ');
    //     if (item == "-1") {
    //       point3D_ids.push_back(kInvalidPoint3DId);
    //     } else {
    //       point3D_ids.push_back(std::stoll(item));
    //     }
    //   }
    // }

    // image.SetUp(Camera(image.CameraId()));
    // image.SetPoints2D(points2D);

    // for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
    //      ++point2D_idx) {
    //   if (point3D_ids[point2D_idx] != kInvalidPoint3DId) {
    //     image.SetPoint3DForPoint2D(point2D_idx, point3D_ids[point2D_idx]);
    //   }
    // }

    // images_.emplace(image.ImageId(), image);
  }
}

// void ColmapReader::ReadPoints3DText(const std::string& path) {
//   points3D_.clear();

//   std::ifstream file(path);
//   CHECK(file.is_open()) << path;

//   std::string line;
//   std::string item;

//   while (std::getline(file, line)) {
//     StringTrim(&line);

//     if (line.empty() || line[0] == '#') {
//       continue;
//     }

//     std::stringstream line_stream(line);

//     // ID
//     std::getline(line_stream, item, ' ');
//     const point3D_t point3D_id = std::stoll(item);

//     // Make sure, that we can add new 3D points after reading 3D points
//     // without overwriting existing 3D points.
//     num_added_points3D_ = std::max(num_added_points3D_, point3D_id);

//     class Point3D point3D;

//     // XYZ
//     std::getline(line_stream, item, ' ');
//     point3D.XYZ(0) = std::stold(item);

//     std::getline(line_stream, item, ' ');
//     point3D.XYZ(1) = std::stold(item);

//     std::getline(line_stream, item, ' ');
//     point3D.XYZ(2) = std::stold(item);

//     // Color
//     std::getline(line_stream, item, ' ');
//     point3D.Color(0) = static_cast<uint8_t>(std::stoi(item));

//     std::getline(line_stream, item, ' ');
//     point3D.Color(1) = static_cast<uint8_t>(std::stoi(item));

//     std::getline(line_stream, item, ' ');
//     point3D.Color(2) = static_cast<uint8_t>(std::stoi(item));

//     // ERROR
//     std::getline(line_stream, item, ' ');
//     point3D.SetError(std::stold(item));

//     // TRACK
//     while (!line_stream.eof()) {
//       TrackElement track_el;

//       std::getline(line_stream, item, ' ');
//       StringTrim(&item);
//       if (item.empty()) {
//         break;
//       }
//       track_el.image_id = std::stoul(item);

//       std::getline(line_stream, item, ' ');
//       track_el.point2D_idx = std::stoul(item);

//       point3D.Track().AddElement(track_el);
//     }

//     point3D.Track().Compress();

//     points3D_.emplace(point3D_id, point3D);
//   }
// }

void ColmapReader::ReadCamerasBinary(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  // CHECK(file.is_open()) << path;

  const size_t num_cameras = ReadBinaryLittleEndian<uint64_t>(&file);

  if (num_cameras > 1)
  {
    std::cerr << "ERROR! ONLY SINGLE CAMERA CASE IS SUPPORTED RIGHT NOW!\n\n";
    std::cerr << "NUMBER OF CAMERAS IN INPUT MODEL : " << num_cameras << "\n\n";
    exit(-1);
  }

  for (size_t i = 0; i < num_cameras; ++i) {
    // class Camera camera;
    // camera.SetCameraId(ReadBinaryLittleEndian<camera_t>(&file));
    // camera.SetModelId(ReadBinaryLittleEndian<int>(&file));
    // camera.SetWidth(ReadBinaryLittleEndian<uint64_t>(&file));
    // camera.SetHeight(ReadBinaryLittleEndian<uint64_t>(&file));
    // ReadBinaryLittleEndian<double>(&file, &camera.Params());
    // CHECK(camera.VerifyParams());
    // cameras_.emplace(camera.CameraId(), camera);
    
    // const uint32_t cam_id = 
    ReadBinaryLittleEndian<uint32_t>(&file);
    const uint32_t model_id = ReadBinaryLittleEndian<uint32_t>(&file);

    if (model_id != 1)
    {
      std::cerr << "ERROR! EXPECTED CAMERA MODEL SHOULD BE : 1 (i.e. PINHOLE in Colmap)!\n\n";
      std::cerr << "INPUT MODEL IS : " << model_id << "\n\n";
      exit(-1);
    }

    m_img_width = ReadBinaryLittleEndian<uint64_t>(&file);
    m_img_height = ReadBinaryLittleEndian<uint64_t>(&file);

    std::vector<double> cam_calib_params(4);
    ReadBinaryLittleEndian<double>(&file, &cam_calib_params);
    
    m_fx = cam_calib_params.at(0);
    m_fy = cam_calib_params.at(1);
    m_cx = cam_calib_params.at(2);
    m_cy = cam_calib_params.at(3);
  }
}

void ColmapReader::ReadImagesBinary(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  // CHECK(file.is_open()) << path;

  const size_t num_reg_images = ReadBinaryLittleEndian<uint64_t>(&file);
  for (size_t i = 0; i < num_reg_images; ++i) {
    // class Image image;

    // image.SetImageId(ReadBinaryLittleEndian<image_t>(&file));
    // const uint32_t img_id = 
    ReadBinaryLittleEndian<uint32_t>(&file);

    // image.Qvec(0) = ReadBinaryLittleEndian<double>(&file);
    // image.Qvec(1) = ReadBinaryLittleEndian<double>(&file);
    // image.Qvec(2) = ReadBinaryLittleEndian<double>(&file);
    // image.Qvec(3) = ReadBinaryLittleEndian<double>(&file);
    // image.NormalizeQvec();

    const double qw = ReadBinaryLittleEndian<double>(&file);
    const double qx = ReadBinaryLittleEndian<double>(&file);
    const double qy = ReadBinaryLittleEndian<double>(&file);
    const double qz = ReadBinaryLittleEndian<double>(&file);

    Eigen::Quaterniond qcw(qw, qx, qy, qz);
    qcw.normalize();

    // image.Tvec(0) = ReadBinaryLittleEndian<double>(&file);
    // image.Tvec(1) = ReadBinaryLittleEndian<double>(&file);
    // image.Tvec(2) = ReadBinaryLittleEndian<double>(&file);

    const double tx = ReadBinaryLittleEndian<double>(&file);
    const double ty = ReadBinaryLittleEndian<double>(&file);
    const double tz = ReadBinaryLittleEndian<double>(&file);

    const Eigen::Vector3d tcw(tx, ty, tz);

    const Sophus::SE3d T_cam_world(qcw, tcw);

    // image.SetCameraId(ReadBinaryLittleEndian<camera_t>(&file));
    // const uint32_t cam_id = 
    ReadBinaryLittleEndian<uint32_t>(&file);

    std::string img_name;

    char name_char;
    do {
      file.read(&name_char, 1);
      if (name_char != '\0') {
        img_name += name_char;
      }
    } while (name_char != '\0');

    m_map_img_names_T_world_cam.emplace(img_name, T_cam_world.inverse());

    const size_t num_points2D = ReadBinaryLittleEndian<uint64_t>(&file);

    // std::vector<Eigen::Vector2d> points2D;
    // points2D.reserve(num_points2D);
    // std::vector<point3D_t> point3D_ids;
    // point3D_ids.reserve(num_points2D);
    for (size_t j = 0; j < num_points2D; ++j) {
      // const double x = ReadBinaryLittleEndian<double>(&file);
      // const double y = ReadBinaryLittleEndian<double>(&file);
      ReadBinaryLittleEndian<double>(&file);
      ReadBinaryLittleEndian<double>(&file);
      // points2D.emplace_back(x, y);
      // point3D_ids.push_back(ReadBinaryLittleEndian<point3D_t>(&file));
      ReadBinaryLittleEndian<uint64_t>(&file);
    }

    // image.SetUp(Camera(image.CameraId()));
    // image.SetPoints2D(points2D);

    // for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
    //      ++point2D_idx) {
    //   if (point3D_ids[point2D_idx] != kInvalidPoint3DId) {
    //     image.SetPoint3DForPoint2D(point2D_idx, point3D_ids[point2D_idx]);
    //   }
    // }

    // image.SetRegistered(true);
    // reg_image_ids_.push_back(image.ImageId());

    // images_.emplace(image.ImageId(), image);
  }
}

// void ColmapReader::ReadPoints3DBinary(const std::string& path) {
//   std::ifstream file(path, std::ios::binary);
//   CHECK(file.is_open()) << path;

//   const size_t num_points3D = ReadBinaryLittleEndian<uint64_t>(&file);
//   for (size_t i = 0; i < num_points3D; ++i) {
//     class Point3D point3D;

//     const point3D_t point3D_id = ReadBinaryLittleEndian<point3D_t>(&file);
//     num_added_points3D_ = std::max(num_added_points3D_, point3D_id);

//     point3D.XYZ()(0) = ReadBinaryLittleEndian<double>(&file);
//     point3D.XYZ()(1) = ReadBinaryLittleEndian<double>(&file);
//     point3D.XYZ()(2) = ReadBinaryLittleEndian<double>(&file);
//     point3D.Color(0) = ReadBinaryLittleEndian<uint8_t>(&file);
//     point3D.Color(1) = ReadBinaryLittleEndian<uint8_t>(&file);
//     point3D.Color(2) = ReadBinaryLittleEndian<uint8_t>(&file);
//     point3D.SetError(ReadBinaryLittleEndian<double>(&file));

//     const size_t track_length = ReadBinaryLittleEndian<uint64_t>(&file);
//     for (size_t j = 0; j < track_length; ++j) {
//       const image_t image_id = ReadBinaryLittleEndian<image_t>(&file);
//       const point2D_t point2D_idx = ReadBinaryLittleEndian<point2D_t>(&file);
//       point3D.Track().AddElement(image_id, point2D_idx);
//     }
//     point3D.Track().Compress();

//     points3D_.emplace(point3D_id, point3D);
//   }
// }

void ColmapReader::DisplayCameraCalib() const {
  std::cout << "\n\n";
  std::cout << "Image Width x Height : " << m_img_width << " x " << m_img_height << " px\n";
  std::cout << "Camera's focal x / y : " << m_fx << " / " << m_fy << "\n";
  std::cout << "Camera's principal point x / y : " << m_cx << " / " << m_cy << "\n"; 
  std::cout << "\n\n";
}

void ColmapReader::DisplayRegisteredImages() const {
  std::cout << "\n\n";
  for (const auto &map_el : m_map_img_names_T_world_cam)
  {
    const std::string &img_name = map_el.first;
    const Sophus::SE3d &T_world_cam = map_el.second;

    std::cout << "Image name : " << img_name << " / Pose is (twc / qwc): ";
    std::cout << T_world_cam.translation().transpose() << " / ";
    std::cout << T_world_cam.unit_quaternion().coeffs().transpose() << "\n";
  }
  std::cout << "\n\n";
}
