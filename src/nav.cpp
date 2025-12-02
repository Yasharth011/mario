#include "nav.hpp"

namespace nav {
struct navContext *setupNav(std::string config_file) {

  struct navContext *nav = new navContext();

  nav->gridmap.loadParameters(config_file);

  return nav;
}

const grid_map::GridMap *
getGridmapFromPointCloud(grid_map::GridMapPclLoader *gridMapPclLoader,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
  gridMapPclLoader->setInputCloud(cloud_ptr);
  gridMapPclLoader->preProcessInputCloud();
  gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
  gridMapPclLoader->addLayerFromInputCloud("elevation"); // add elevation layer

  return &(gridMapPclLoader->getGridMap());
}
} // namespace nav
