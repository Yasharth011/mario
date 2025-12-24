#ifndef NAV_HPP
#define NAV_HPP

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rerun/recording_stream.hpp>

namespace ob = ompl::base;

namespace nav {

struct parameters {
  float grid_map_dim[2];
  float grid_map_res;
  float occupancy_threshold[2];
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
  std::vector<grid_map::Index> occupancy_list;
  ob::StateSpacePtr space;
  ob::SpaceInformationPtr si;
};

parameters loadParameters(const std::string &filename);

struct navContext *setupNav(std::string config_file);

void preProcessPointCloud(navContext *ctx,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr rawInputCloud,
                          const Eigen::Matrix<double, 4, 4> T_matrix);

void processGridMapCells(navContext *ctx,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

class ValidityChecker : public ob::StateValidityChecker {
public:
  navContext *nav_ctx;
  ValidityChecker(const ob::SpaceInformationPtr &si, navContext *ctx)
      : ob::StateValidityChecker(si), nav_ctx(ctx) {}

  bool isValid(const ob::State *state) const;
  double clearance(const ob::State *state) const;
};
} // namespace nav
#endif
