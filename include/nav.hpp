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

struct navContext{
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
};

float grid_resolution = 0.001f;
int batch_threshold = 1; 


std::vector<Eigen::Vector3f>
processPointCloud(std::vector<Eigen::Vector3f> raw_points);

void updateMaps(const std::vector<Eigen::Vector3f> &points);

std::vector<Node> prunePath(const std::vector<Node>& path);

bool findPath(const Node& current_start, const Node& current_goal, std::vector<Node>& dense_path);
} // namespace mapping
#endif
