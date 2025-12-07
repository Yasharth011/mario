#ifndef NAV_HPP
#define NAV_HPP

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_pcl/grid_map_pcl.hpp>
#include <pcl/point_cloud.h>
#include <rerun/recording_stream.hpp>

namespace nav {

struct parameters {
  float grid_map_dim[2];
  float grid_map_res;
  std::string layer_name;
  std::string frame_id;
  std::map<std::string, std::array<float, 2>> pass_filter;
  float voxel_leaf_size[3];
  float min_filtering_points;
  float min_grid_points;
};

struct navContext {
  grid_map::GridMap *map;
  struct parameters params;
  std::string layer;
};

parameters loadParameters(const std::string &filename);

struct navContext *setupNav(std::string config_file);

void preProcessPointCloud(navContext *ctx,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr rawInputCloud,
                          const Eigen::Matrix<double, 4, 4> T_matrix);

void processGridMapCells(navContext *ctx,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
} // namespace nav
#endif
