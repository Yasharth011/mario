#include "nav.hpp"
#include <cmath>
#include <grid_map_core/TypeDefs.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

namespace nav {

parameters loadParameters(const std::string &filename) {
  parameters params;
  YAML::Node config = YAML::LoadFile(filename);

  // Parse Grid Map settings
  if (config["grid_map"]) {
    const YAML::Node &gm = config["grid_map"];
    params.grid_map_dim[0] = gm["x"].as<float>();
    params.grid_map_dim[1] = gm["y"].as<float>();
    params.grid_map_res = gm["resolution"].as<float>();
    params.min_grid_points = gm["min_grid_points"].as<int>();
  }

  // Parse Filters
  if (config["filters"]) {
    const YAML::Node &filters = config["filters"];

    params.min_filtering_points = filters["min_filtering_points"].as<int>();

    // Passthrough Filter (Map of arrays)
    if (filters["passthrough_filter"]) {
      const YAML::Node &pt = filters["passthrough_filter"];

      for (YAML::const_iterator it = pt.begin(); it != pt.end(); ++it) {
        std::string axis = it->first.as<std::string>();
        float min_val = it->second["min_limit"].as<float>();
        float max_val = it->second["max_limit"].as<float>();

        params.pass_filter[axis] = {min_val, max_val};
      }
    }

    // Voxel Filter
    if (filters["voxel_size"]) {
      const YAML::Node &vs = filters["voxel_size"];
      params.voxel_leaf_size[0] = vs["x"].as<float>();
      params.voxel_leaf_size[1] = vs["y"].as<float>();
      params.voxel_leaf_size[2] = vs["z"].as<float>();
    }
  }

  return params;
}

struct navContext *setupNav(std::string config_file) {

  struct navContext *nav(new navContext());

  nav->params = loadParameters(config_file);

  nav->layer = "elevation";

  nav->map = new grid_map::GridMap({nav->layer});

  nav->map->setFrameId("cost_map");

  nav->map->setGeometry(grid_map::Length(nav->params.grid_map_dim[0],
                                         nav->params.grid_map_dim[1]),
                        nav->params.grid_map_res);

  return nav;
}

void preProcessPointCloud(navContext *ctx,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr rawInputCloud,
                          const Eigen::Matrix<double, 4, 4> T_matrix) {

  // transform pcl cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*rawInputCloud, *transformed_cloud, T_matrix);

  // filtering
  if (rawInputCloud->size() < ctx->params.min_filtering_points) {
    spdlog::info(
        std::format("Skipping filters: Pointcloud size ({}) is below minimum "
                    "threshold ({})",
                    rawInputCloud->size(), ctx->params.min_filtering_points));
    return;
  }

  spdlog::info(
      std::format("Before filtering: {} points", rawInputCloud->size()));

  pcl::PassThrough<pcl::PointXYZ> passthrough_filter;

  for (auto filter : ctx->params.pass_filter) {
    passthrough_filter.setInputCloud(rawInputCloud);
    passthrough_filter.setFilterFieldName(filter.first);
    passthrough_filter.setFilterLimits(filter.second[0], filter.second[1]);
    passthrough_filter.filter(*rawInputCloud);
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(rawInputCloud);
  voxel_filter.setLeafSize(ctx->params.voxel_leaf_size[0],
                           ctx->params.voxel_leaf_size[1],
                           ctx->params.voxel_leaf_size[2]);
  voxel_filter.filter(*rawInputCloud);

  spdlog::info(
      std::format("After filtering: {} points", rawInputCloud->size()));
};

void processGridMapCells(navContext *ctx,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud) {

  if (pointCloud->points.size() < ctx->params.min_grid_points) {
    spdlog::info(std::format(
        "Skipping Elevation Layer: Pointcloud size ({}) is below minimum "
        "threshold ({})",
        pointCloud->size(), ctx->params.min_grid_points));
    return;
  }

  grid_map::Matrix &elevation_data = ctx->map->get(ctx->layer);

  for (const auto &point : pointCloud->points) {

    grid_map::Position position(point.x, point.y);

    grid_map::Index index;
    if (ctx->map->getIndex(position, index)) {
      float &cell_height = elevation_data(index(0), index(1));

      if (std::isnan(cell_height) || point.z > cell_height) {
        cell_height = point.z;
      }
    }
  }
}
} // namespace nav
