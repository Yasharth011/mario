#ifndef NAV_HPP
#define NAV_HPP

#include <Eigen/Dense>
#include <cmath>
#include <mapping.h>
#include <pathplanning.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sbpl/headers.h>
#include <quadtree.h>
#include <rerun/recording_stream.hpp>

namespace nav {

inline int batch_threshold = 1;
inline float grid_resolution = 0.02f;
inline float height = 1.0f;
inline float proxfactor = 0.2f;
inline int map_limit = 20;
inline mapping::Point center;
inline float rootSize;

struct navContext {
  mapping::Gridmap gridmap;
  quadtree::QuadtreeNode lowQuadtree;
  quadtree::QuadtreeNode midQuadtree;
  quadtree::QuadtreeNode highQuadtree;
  planning::Node start;
  planning::Node goal;
  planning::Node current_start;
  planning::Node current_goal;
  planning::Node final_goal;
  std::vector<planning::Node> full_path;
  std::set<std::pair<int, int>> visited_nodes;
  std::set<std::pair<int, int>> failed_goals;
  std::deque<planning::Node> recent_goals;
  std::vector<planning::Node> dense_path;

  navContext()
      : gridmap(), lowQuadtree(center, rootSize, 1),
        midQuadtree(center, rootSize, 1), highQuadtree(center, rootSize, 1),
        start(0, 0), goal(0, 0), current_start(0, 0), current_goal(0, 0),
        final_goal(0, 0), full_path(), visited_nodes(), failed_goals(),
        recent_goals(), dense_path() {}
};

std::vector<Eigen::Vector3f>
processPointCloud(std::vector<Eigen::Vector3f> raw_points);

void updateMaps(struct navContext *ctx, struct mapping::Slam_Pose &pose,
                const std::vector<Eigen::Vector3f> &points,
                const rerun::RecordingStream &rec);

std::vector<planning::Node> prunePath(const std::vector<planning::Node> &path);

bool findCurrentGoal(nav::navContext *ctx);

bool findPath(navContext *ctx, std::vector<planning::Node> &dense_path);

void log_gridmap(struct navContext *ctx, float grid_resolution,
                 const rerun::RecordingStream &rec,
                 const struct mapping::Slam_Pose &pose);
} // namespace nav
#endif
