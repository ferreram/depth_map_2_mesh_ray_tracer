#include <iostream>

#include "colmap_reader.h"
#include "image_to_rays.h"
#include "mesh_raycaster.h"

int main(int argc, char *argv[])
{
  if (argc < 5)
  {
    std::cerr << "Usage : ./render_images_from_mesh mesh_path mesh_tex_img_path colmap_result_path out_rendered_img_path save_as_png(default = true) filter_from_normals(default = true)" << std::endl;
    return -1;
  }

  // 1. Get args
  // =============================================
  std::string mesh_path = argv[1];
  std::string mesh_tex_img_path = argv[2];
  std::string colmap_result_path = argv[3];
  std::string out_rendered_img_path = argv[4];

  bool save_as_png = true;
  bool filter_from_normals = true;
  if (argc > 5)
  {
    save_as_png = std::stoi(argv[5]);

    if (argc > 6)
      filter_from_normals = std::stoi(argv[6]);
  }

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
  RenderImagesFromMesh(mesh_path, 
                      mesh_tex_img_path,
                      out_rendered_img_path, 
                      colmap_reader.m_map_img_names_T_world_cam, 
                      image_rays.m_vbvs, 
                      image_rays.m_img_cols, 
                      image_rays.m_img_rows,
                      filter_from_normals,
                      save_as_png);

  return 0;
}