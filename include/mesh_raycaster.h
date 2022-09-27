#pragma once

#include "mesh_loader.h"

#include "acc/bvh_tree.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/optflow.hpp>

#include <opencv2/highgui.hpp>

void RaycastImagesToMesh(const std::string& _mesh_path,
                        const std::string& _out_depth_map_path, 
                        const std::unordered_map<std::string, Sophus::SE3d>& _map_img_names_T_world_cam,
                        const VectorAlignV3f& _vbvs,
                        const int _img_width,
                        const int _img_height,
                        const bool _filter_depth_on_normals = true)
{
  // 1. Load Mesh file
  std::cout << "\nLoading mesh from file : " << _mesh_path << "\n\n";
  
  std::vector<float3> vec_verts;
  std::vector<float3> vec_normals;
  std::vector<uint3> vec_faces;

  readPlyFile(_mesh_path, vec_verts, vec_faces, vec_normals);

  // If user do not want to filter on normals, behave as if there was no normals
  // in input mesh
  const bool has_normals = !vec_normals.empty() && _filter_depth_on_normals;

  if (!has_normals && _filter_depth_on_normals)
  {
    std::cout << "\n\nWARNING!!! No normals in provided mesh.";
    std::cout << "Depth maps won't be filtered from normals!\n\n";
  }

  // 2. Set up BVH vertices and faces from the Mesh
  std::vector<math::Vec3f> vertices;
  std::vector<math::Vec3f> normals;
  std::vector<size_t> faces;
  vertices.reserve(vec_verts.size());
  faces.reserve(vec_faces.size() * 3);

  for (const auto &ver : vec_verts)
  {
    vertices.push_back(math::Vec3f(ver.x, ver.y, ver.z));
  }

  for (const auto &face : vec_faces)
  {
    faces.push_back(face.v1);
    faces.push_back(face.v2);
    faces.push_back(face.v3);
  }

  if (has_normals)
  {
    normals.reserve(vec_normals.size());
    for (const auto &n : vec_normals)
    {
      normals.push_back(math::Vec3f(n.x, n.y, n.z));
    }
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
  for (size_t i = 0; i < nb_poses; ++i)
  {
    // Get image to process
    const Sophus::SE3f& T_world_cam = _map_img_names_T_world_cam.at(*v_img_names[i]).cast<float>();
    const Sophus::SE3f T_cam_world = T_world_cam.inverse();

    // Create associated depth map
    cv::Mat depth_img = cv::Mat::zeros(_img_height, _img_width, CV_16U);
    auto depth_val = depth_img.begin<uint16_t>();

    for (size_t j = 0; j < nb_bvs; ++j, ++depth_val)
    {
      // Create ray object
      acc::Ray ray;

      // Get origin of ray and direction in world frame
      const Eigen::Vector3f& origin = T_world_cam.translation();
      const Eigen::Vector3f dir = T_world_cam.so3() * _vbvs[j];

      ray.origin = math::Vec3f(origin[0], origin[1], origin[2]);
      ray.dir = math::Vec3f(dir[0], dir[1], dir[2]);
      
      ray.tmin = 0.0f;
      ray.tmax = std::numeric_limits<float>::max();

      // Raycast ray onto mesh
      acc::BVHTree::Hit hit;
      if (bvhtree.intersect(ray, &hit))
      {
        if (has_normals)
        {
          const math::Vec3f& w = hit.bcoords;

          const auto& n0 = normals.at(faces.at(hit.idx * 3 + 0));
          const auto& n1 = normals.at(faces.at(hit.idx * 3 + 1));
          const auto& n2 = normals.at(faces.at(hit.idx * 3 + 2));

          Eigen::Vector3f n;
          for (size_t k=0; k < 3; ++k)
          {
            n[k] = n0[k] * w[0] + n1[k] * w[1] + n2[k] * w[2];
          }

          const Eigen::Vector3f cam_normal = T_cam_world.so3() * n.normalized();

          if (cam_normal[2] < 0.f) // if normal is looking toward the camera
          {
            *depth_val = static_cast<uint16_t>((1000.f * hit.t * _vbvs[j])[2]);
          }
        }
        else
        {
          *depth_val = static_cast<uint16_t>((1000.f * hit.t * _vbvs[j])[2]);
        }
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

