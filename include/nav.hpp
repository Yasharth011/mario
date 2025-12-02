#ifndef NAV_HPP
#define NAV_HPP

#include <Eigen/Dense>
#include <cmath>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_pcl/grid_map_pcl.hpp>
#include <pcl/point_cloud.h>
#include <rerun/recording_stream.hpp>

namespace nav {
struct navContext{
	struct grid_map::GridMapPclLoader gridmap;
};

struct navContext *setupNav(std::string config_file);

const grid_map::GridMap *
getGridmapFromPointCloud(grid_map::GridMapPclLoader *gridMapPclLoader,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
} // namespace nav
#endif
