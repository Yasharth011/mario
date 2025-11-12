#include "nav.hpp"
#include <mapping.h>
#include <pathplanning.h>
#include <quadtree.h>

namespace nav {

std::vector<Eigen::Vector3f>
processPointCloud(std::vector<Eigen::Vector3f> raw_points) {

  auto pcl_cloud = mapping::convert_to_pcl(raw_points);

  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcl_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(0.2, 4.0);
  passthrough.filter(*pcl_cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(pcl_cloud);
  voxel.setLeafSize(0.02f, 0.02f, 0.02f);
  voxel.filter(*pcl_cloud);

  std::vector<Eigen::Vector3f> filtered_points;
  filtered_points.reserve(pcl_cloud->points.size());
  for (const auto &point : pcl_cloud->points) {
    filtered_points.emplace_back(point.x, point.y, point.z);
  }

  return filtered_points;
}

void updateMaps(struct navContext *ctx, struct mapping::Slam_Pose &pose,
                const std::vector<Eigen::Vector3f> &points,
                const rerun::RecordingStream &rec) {
  mapping::create_gridmap(ctx->gridmap, points, pose, grid_resolution, height, proxfactor);

  quadtree::updateQuadtreesWithPointCloud(&(ctx->lowQuadtree),
                                          &(ctx->midQuadtree),
                                          &(ctx->highQuadtree), points, pose);

  if (ctx->gridmap.occupancy_grid.size() >= batch_threshold) {
    nav::log_gridmap(ctx, grid_resolution, rec, pose);
    batch_threshold += ctx->gridmap.occupancy_grid.size();
  }
}

std::vector<planning::Node> prunePath(const std::vector<planning::Node> &path) {
  if (path.empty())
    return {};
  std::vector<planning::Node> pruned_path;
  pruned_path.push_back(path[0]);
  for (size_t i = 1; i < path.size(); ++i) {
    if (!(path[i] == path[i - 1])) {
      pruned_path.push_back(path[i]);
    }
  }
  return pruned_path;
}

bool findPath(struct navContext *ctx) {
  std::vector<planning::Node> sparse_path =
      astarsparse(ctx->gridmap, ctx->current_start, ctx->current_goal);

  if (sparse_path.empty()) {
    ctx->dense_path = planning::astarquad(
        &(ctx->lowQuadtree), &(ctx->midQuadtree), &(ctx->highQuadtree),
        ctx->current_start, ctx->current_goal, 1.0f);
  } else {
    ctx->dense_path.push_back(sparse_path[0]);

    for (size_t i = 1; i < sparse_path.size(); ++i) {
      std::vector<planning::Node> segment = planning::astarquad(
          &(ctx->lowQuadtree), &(ctx->midQuadtree), &(ctx->highQuadtree),
          sparse_path[i - 1], sparse_path[i], 1.0f);

      if (!segment.empty()) {
        ctx->dense_path.insert(ctx->dense_path.end(), segment.begin() + 1,
                               segment.end());
      } else {
      }
    }
  }
  return !ctx->dense_path.empty();
}

void log_gridmap(struct navContext *ctx, float grid_resolution,
                 const rerun::RecordingStream &rec,
                 const mapping::Slam_Pose &pose) {
  float min_x = (pose.x - 5.0f) / grid_resolution;
  float max_x = (pose.x + 5.0f) / grid_resolution;
  float min_y = (pose.y - 5.0f) / grid_resolution;
  float max_y = (pose.y + 5.0) / grid_resolution;
  float scale_factor = 1000.0f; // Put 1000 so that it is in mm
  std::cout << "Occupancy grid size: " << ctx->gridmap.occupancy_grid.size()
            << std::endl;
  if (ctx->gridmap.occupancy_grid.empty()) {
    std::cout << "Error: Occupancy grid is empty!" << std::endl;
  } else {
    std::cout << "Occupancy grid has data!" << std::endl;
  }
  std::vector<rerun::Color> colors;
  std::vector<rerun::Position3D> roverposition;

  for (const auto &entry : ctx->gridmap.occupancy_grid) {
    const auto &[coord, value] = entry;
    float grid_x = coord.first;
    float grid_y = coord.second;

    rerun::Color color = mapping::get_color_for_cost(value);

    std::vector<rerun::Position3D> points = {
        rerun::Position3D{grid_x, grid_y, 0.0f}};

    colors.push_back(color);
    std::string cell_id = "gridcell_(" + std::to_string(grid_x) + "," +
                          std::to_string(grid_y) + ")_" +
                          std::to_string(value.cost);
    std::string tag = "grid_map" + cell_id;
    rec.log(tag,
            rerun::Points3D(points).with_colors({color}).with_radii({0.5f}));
  }
  colors.clear();
}
} // namespace nav
