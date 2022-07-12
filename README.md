# depth_map_2_mesh_ray_tracer

Raytracing images onto mesh from SfM results to compute depth maps.

Right now, this code uses the output of Colmap to get camera's poses and camera's calibration (undistorted images are expected, i.e. PINHOLE model).

1. Build thirdparty libs : 

$ chmod +x build_thridpary.sh
= ./build_thirdparty.sh

2. Build the code :

$ mkdir build 
$ cd build
$ cmake ..
$ make -j4

3. Run :

$ cd build
$ ./ply_mesh_to_depth_maps path1/to/colmap_mesh.ply path2/to/colmap_sparse_results/ output/path/for/depth_maps/

