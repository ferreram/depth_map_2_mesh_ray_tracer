#include <iostream>

#include "colmap_reader.h"
#include "image_to_rays.h"
#include "mesh_raycaster.h"

int main(int argc, char *argv[])
{
  if (argc < 4)
  {
    std::cerr << "Usage : ./ply_mesh_to_depth_maps mesh_path colmap_result_path out_depth_maps_path" << std::endl;
    return -1;
  }

  // 1. Get args
  // =============================================
  std::string mesh_path = argv[1];
  std::string colmap_result_path = argv[2];
  std::string out_depth_maps_path = argv[3];

  // 2. Load Camera's Calib (Expecting Undistorted Calib) and Images' SE(3) Poses
  // ============================================================================
  ColmapReader colmap_reader;
  colmap_reader.Read(colmap_result_path);

  colmap_reader.DisplayCameraCalib();
  colmap_reader.DisplayRegisteredImages();

  // 3.  Init Pixel to Rays from Calib
  // ==========================================================
  ImageRays image_rays(colmap_reader.m_fx, colmap_reader.m_fy, colmap_reader.m_cx, colmap_reader.m_cy,
                      colmap_reader.m_img_width, colmap_reader.m_img_height);

  // 4. Load mesh file and Init BVH-tree + Raycast every images onto Mesh
  // + Save the resulting Depth Maps!
  // ====================================================================
  RaycastImagesToMesh(mesh_path, 
                      out_depth_maps_path, 
                      colmap_reader.m_map_img_names_T_world_cam, 
                      image_rays.m_vbvs, 
                      image_rays.m_img_cols, 
                      image_rays.m_img_rows);

  return 0;
}