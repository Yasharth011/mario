#ifndef MAPPING_HPP
#define MAPPING_HPP

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <librealsense2/rs.hpp>

#include <vector>

struct CellCost {
    float cost;
    float proxcost;
    bool visited;
    bool proxvisited;

    CellCost(float c = 0.0f, float pc = 0.0f, bool v = false, bool p = false)
        : cost(c), proxcost(pc), visited(v), proxvisited(p) {}
};

struct pair_hash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1); // Combine the two hashes
    }
};

// Define the Gridmap structure
struct Gridmap {
    std::unordered_map<std::pair<int, int>, CellCost, pair_hash> occupancy_grid;
    float min_x, min_y, max_x, max_y;

    Gridmap() : min_x(0), min_y(0), max_x(0), max_y(0) {}

    Gridmap(std::unordered_map<std::pair<int, int>, CellCost, pair_hash> grid,
            float min_x, float min_y, float max_x, float max_y)
        : occupancy_grid(grid), min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y) {}
};

struct Pose {
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Quaternionf orientation;
};



// Function declaration for creating a grid map
void create_gridmap(Gridmap &gridmap, const std::vector<Eigen::Vector3f> &point_vectors,
                    const Pose &roverpose, float grid_resolution, float height, float prox_factor);

pcl::PointCloud<pcl::PointXYZ>::Ptr
convert_to_pcl(const std::vector<Eigen::Vector3f> &point_vectors);

void update_rover_pose(Pose &pose, const Eigen::Vector3f &accel_data,
                       const Eigen::Vector3f &gyro_data, float delta_time);

Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec);

#endif
