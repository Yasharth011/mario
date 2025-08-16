#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>

// Define the Gridmap structure
struct Gridmap {
  std::vector<std::vector<bool>> occupancy_grid;
  float min_x, min_y, max_x, max_y;
};

struct Pose {
  Eigen::Quaternionf orientation;
  Eigen::Vector3f position;
};

// Function declaration for creating a grid map
Gridmap create_gridmap(const std::string &ply_file, float grid_resolution,
                       float height = 2.0);

pcl::PointCloud<pcl::PointXYZ>::Ptr
convert_to_pcl(const std::vector<Eigen::Vector3f> &point_vectors);

void update_rover_pose(Pose &pose, const Vector3f &accel_data,
                       const Vector3f &gyro_data, float delta_time);

#endif // GRIDMAP_H
