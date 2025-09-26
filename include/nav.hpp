#ifndef NAV_HPP
#define NAV_HPP

#include <Eigen/Dense>
#include <cmath>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <quadtree.h>
#include <mapping.h>
#include <pathplanning.h>

namespace nav {

inline int batch_threshold = 1; 
inline Point center;
inline float rootSize;

struct navContext {
  Gridmap gridmap;
  QuadtreeNode lowQuadtree;
  QuadtreeNode midQuadtree;
  QuadtreeNode highQuadtree;
  Node start;
  Node goal;
  Node current_start;
  Node current_goal;
  Node final_goal;
  std::vector<Node> full_path;
  std::vector<std::pair<int, int>> visited_node;
  std::set<std::pair<int, int>> failed_goals;
  std::vector<Node> dense_path;

  navContext()
      : gridmap(), lowQuadtree(center, rootSize, 1),
        midQuadtree(center, rootSize, 1), highQuadtree(center, rootSize, 1),
        start(0, 0), goal(0, 0), current_start(0, 0), current_goal(0, 0), final_goal(0, 0),
        full_path(), visited_node(), failed_goals(), dense_path() {}
};

std::vector<Eigen::Vector3f>
processPointCloud(std::vector<Eigen::Vector3f> raw_points);

void updateMaps(struct navContext *ctx, struct Slam_Pose &pose,
                const std::vector<Eigen::Vector3f> &points);

std::vector<Node> prunePath(const std::vector<Node>& path);

bool findPath(const Node& current_start, const Node& current_goal, std::vector<Node>& dense_path);
} // namespace mapping
#endif
