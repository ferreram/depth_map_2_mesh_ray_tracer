#pragma once

#include "mesh_loader.h"

#include "acc/bvh_tree.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/optflow.hpp>

void RaycastImagesToMesh(const std::string& _mesh_path,
                        const std::string& _out_depth_map_path, 
                        const std::unordered_map<std::string, Sophus::SE3d>& _map_img_names_T_world_cam,
                        const VectorAlignV3d& _vbvs,
                        const int _img_width,
                        const int _img_height)
{
  // 1. Load Mesh file
  std::cout << "\nLoading mesh from file : " << _mesh_path << "\n\n";
  
  std::vector<float3> vec_verts;
  std::vector<uint3> vec_faces;

  readPlyFile(_mesh_path, vec_verts, vec_faces);

  // 2. Set up BVH vertices and faces from the Mesh
  std::vector<math::Vec3f> vertices;
  std::vector<size_t> faces;
  vertices.reserve(vec_verts.size());
  faces.reserve(vec_faces.size() * 3);

  for (const auto &ver : vec_verts)
  {
    vertices.push_back(math::Vec3f(ver.x, ver.y, ver.z));
    // std::cout << "Adding vertex : " << vertices.back() << std::endl;
  }

  for (const auto &face : vec_faces)
  {
    faces.push_back(face.v1);
    faces.push_back(face.v2);
    faces.push_back(face.v3);
    // std::cout << "Faces from vert idx : " << face.v1 << " " << face.v2 << " " << face.v3 << std::endl;
  }

  std::cout << "Creating a BVH tree from #" << faces.size() / 3;
  std::cout << " faces and #" << vertices.size() << " vertices!" << std::endl;

  acc::BVHTree bvhtree(faces, vertices);

  // Number of poses to look for and 
  // number of pixels to raytrace per pose
  const size_t nb_poses = _map_img_names_T_world_cam.size();
  const size_t nb_bvs = _vbvs.size();

  // Store img names into vector (trick to allow the use of openMP)
  std::vector<const std::string*> v_img_names;
  v_img_names.reserve(_map_img_names_T_world_cam.size());
  
  for (const auto &map_el : _map_img_names_T_world_cam)
  {
    v_img_names.push_back(&map_el.first);
  }

  manual_timer timer;
  timer.start();

  #pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < nb_poses; i++)
  {
    // Get image to process
    const Sophus::SE3d& T_world_cam = _map_img_names_T_world_cam.at(*v_img_names[i]);

    // Create associated depth map
    cv::Mat depth_img = cv::Mat::zeros(_img_height, _img_width, CV_16U);
    auto depth_val = depth_img.begin<uint16_t>();
    
    const uint16_t zero_u16 = 0;

    for (size_t j = 0; j < nb_bvs; j++, depth_val++)
    {
      // Create ray object
      acc::Ray ray;

      // Get origin of ray and direction in world frame
      const Eigen::Vector3d origin = T_world_cam.translation();
      const Eigen::Vector3d dir = T_world_cam.so3() * _vbvs[j];

      ray.origin = math::Vec3f(origin[0], origin[1], origin[2]);
      ray.dir = math::Vec3f(dir[0], dir[1], dir[2]);
      
      ray.tmin = 0.0F;
      // ray.tmin = std::numeric_limits<float>::lowest();
      ray.tmax = std::numeric_limits<float>::max();

      // Raycast ray onto mesh
      acc::BVHTree::Hit hit;
      if (bvhtree.intersect(ray, &hit))
      {
        // Eigen::Vector3d campt_bv = hit.t * _vbvs.at(j);
        *depth_val = static_cast<uint16_t>((1000.f * hit.t * _vbvs[j])[2]);
      }
    }

    const size_t last_idx = v_img_names[i]->find_last_of(".");
    const std::string img_prefix = v_img_names[i]->substr(0, last_idx);
    const std::string out_path = _out_depth_map_path + "/depth_" + img_prefix + ".png";
    cv::imwrite(out_path, depth_img);

    std::cout << "\nDepth Map : " << out_path << " written!\n";    
  }

  timer.stop();

  std::cout << "\n >>> #" << nb_poses << " images raycasted in " << timer.get() / 1000.f << " s!\n";

  std::cout << "\n DONE!!! \n";
}

