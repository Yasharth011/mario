#include "nav.hpp"

namespace nav {

std::vector<Eigen::Vector3f>
processPointCloud(std::vector<Eigen::Vector3f> raw_points) {

  auto pcl_cloud = convert_to_pcl(raw_points);

  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcl_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(0.5, 0.5);
  passthrough.filter(*pcl_cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(pcl_cloud);
  voxel.setLeafSize(0.05f, 0.05f, 0.05f);
  voxel.filter(*pcl_cloud);

  std::vector<Eigen::Vector3f> filtered_points;
  filtered_points.reserve(pcl_cloud->points.size());
  for (const auto &point : pcl_cloud->points) {
    filtered_points.emplace_back(point.x, point.y, point.z);
  }

  return filtered_points;
}

void updateMaps(struct navContext *ctx, struct Slam_Pose &pose,
                const std::vector<Eigen::Vector3f> &points) {
  create_gridmap(ctx->gridmap, points, pose);

  updateQuadtreesWithPointCloud(&(ctx->lowQuadtree), &(ctx->midQuadtree),
                                &(ctx->highQuadtree), points, pose);

  if (ctx->gridmap.occupancy_grid.size() >= batch_threshold) {
    batch_threshold += ctx->gridmap.occupancy_grid.size();
  }
}

std::vector<Node> prunePath(const std::vector<Node> &path) {
  if (path.empty())
    return {};
  std::vector<Node> pruned_path;
  pruned_path.push_back(path[0]);
  for (size_t i = 1; i < path.size(); ++i) {
    if (!(path[i] == path[i - 1])) {
      pruned_path.push_back(path[i]);
    }
  }
  return pruned_path;
}

bool findPath(struct navContext *ctx) {
  std::vector<Node> sparse_path =
      astarsparse(ctx->gridmap, ctx->current_start, ctx->current_goal);

  if (sparse_path.empty()) {
    ctx->dense_path = astarquad(&(ctx->lowQuadtree), &(ctx->midQuadtree),
                                &(ctx->highQuadtree), ctx->current_start,
                                ctx->current_goal, 1.0f);
  } else {
    ctx->dense_path.push_back(sparse_path[0]);

    for (size_t i = 1; i < sparse_path.size(); ++i) {
      std::vector<Node> segment = astarquad(
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
} // namespace nav
