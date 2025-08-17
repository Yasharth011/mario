#include "mapping.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

using namespace std;
using namespace pcl;
using namespace Eigen;

void create_gridmap(Gridmap &gridmap, const vector<Vector3f> &point_vectors,
                    const Pose &roverpose, float grid_resolution, float height,
                    float proxfactor)
{
  float boundary_threshold = 0.01f;
  if (roverpose.position.x() < gridmap.min_x + boundary_threshold) {
    gridmap.min_x -= (grid_resolution * 50.0f);
  }
  if (roverpose.position.x() > gridmap.max_x - boundary_threshold) {
    gridmap.max_x += (grid_resolution * 50.0f);
  }
  if (roverpose.position.y() < gridmap.min_y + boundary_threshold) {
    gridmap.min_y -= (grid_resolution * 50.0f);
  }
  if (roverpose.position.y() > gridmap.max_y - boundary_threshold) {
    gridmap.max_y += (grid_resolution * 50.0f);
  }

  unordered_map<pair<int, int>, CellCost, pair_hash> updated_occupancy_grid =
      gridmap.occupancy_grid;
  int proxradius = 3;

  float rover_x = roverpose.position.x();
  float rover_y = roverpose.position.y();
  // ORIENTATION theta

  // to verify if it is valid
  // cout<<"Rover position (real-world): ("<<rover_x<<", "<<rover_y<<")"<<endl;
  for (const auto &point : point_vectors) {

    // cout << "Raw point data: (" << point.x() << ", " << point.y() << ", " <<
    // point.z() << ")" << endl;
    float dx = point.z();  // forward motion)
    float dy = -point.x(); // X becomes -Y
    float dz = -point.y();

    float theta = atan2(
        2.0f * (roverpose.orientation.w() * roverpose.orientation.z() +
                roverpose.orientation.x() * roverpose.orientation.y()),
        1.0f - 2.0f * (roverpose.orientation.y() * roverpose.orientation.y() +
                       roverpose.orientation.z() * roverpose.orientation.z()));

    float ned_theta = -(theta - M_PI_2);

    float rotated_x = cos(ned_theta) * dx - sin(ned_theta) * dy;
    float rotated_y = sin(ned_theta) * dx + cos(ned_theta) * dy;

    int grid_x = static_cast<int>(rotated_x / 1.0f); // grid coordinates
    int grid_y = static_cast<int>(rotated_y / 1.0f);

    // Height remains the same, assuming you're checking for obstacles based on
    // height
    float height_at_point = dz;

    // Debugging output
    // *********************UNCOMMENT THE PRINT STATEMENTS IF
    // NEEDED******************//
    // cout << "Mapped grid position: (" << grid_x << ", " << grid_y << ")" <<
    // endl; cout << "Height at (" << grid_x << " , " << grid_y << ") -> " <<
    // height_at_point << "\n" << endl;
    /******************************************/

    // TO UPDATE BASED ON THE HIGHEST POINT DETECTED
    //	std::map<std::pair<int, int>, std::pair<float, float>> height_map; //
    //{min, max}
    /*std::unordered_map<std::pair<int, int>, std::pair<float, float>,
    pair_hash> height_map;  // {min_height, max_height} std::pair<int, int>
    grid_cell = {grid_x, grid_y};


           auto it = height_map.find(grid_cell);
    if (it == height_map.end()) {
        if (height_map.find(grid_cell) == height_map.end()) {

      height_map.emplace(grid_cell, std::make_pair(height_at_point,
    height_at_point)); } else { it->second.first = std::min(it->second.first,
    height_at_point); it->second.second = std::max(it->second.second,
    height_at_point);
    }
    }*/

    float adjusted_height = height_at_point + 0.; // Adjust by 30 cm (0.3m)
    //    float adjusted_height = height_map[grid_cell].second;

    // cout<<"Real height = "<<height_at_point<<" & Height adjusted:
    // "<<adjusted_height<<"\n"<<endl;

    float cost = 0.0f;
    if (adjusted_height > height) {
      cost = 10.0f; // very high=cant go cost is from range 0 to 10
    } else if (adjusted_height > (height / 2) && adjusted_height <= (height)) {
      cost = 5.0f;
    } else if (adjusted_height > (height / 4) &&
               adjusted_height <= (height / 2)) {
      cost = 1.0f;
    }

    //	  std::cout << "Mapped grid position: (" << grid_x  << ", " << grid_y <<
    //"), COST ->" <<cost<<"\n"<< std::endl;

    // USE BIT MASK ENCODING TO CHECK IF THE NODE IS VISITED OR NOT, I have used
    // visited and proxvisited boolean visited and proxvisited with the cost
    // proxcost.
    pair<int, int> current = {grid_x, grid_y};
    // std::cout << "Before updating: (" << current.first << ", " <<
    // current.second << ") -> Cost: " << cost << std::endl;
    CellCost &cell = updated_occupancy_grid[current];

    float proxcostupdate = cell.proxcost;
    bool proxvupdate = cell.proxvisited;

    if (!cell.visited && !cell.proxvisited) { // CHECK HERE
      // update the cost AND mark them as visited to avoid re-iteration of that
      // cell
      cell.cost += cost; // adding new cost to the existing cost (the existing
                         // cost might be proximity cost= proxcost)
      cell.visited = true; // mark that cell as visited to avoid reiteration
      cell.proxvisited = proxvupdate;
      if (cell.cost == 0.0f) {
        updated_occupancy_grid.erase(current);
      }
    } else {
      // if not found in the occupancy grid then visit that node and then update
      // it
      if (cost > 0.0f) {
        updated_occupancy_grid[current] =
            CellCost(cost, proxcostupdate, true, false);
      } // CellCost(float c = 0.0f, pc=0.0f, bool v = false, bool p = false) :
        // cost(c),proxcost,(pc), visited(v),proxvisited(p)
    }
    // cout<< "After Updating: ("<< current.first<< ", "<< current.second<< ")
    // -> Cost: "<< cost<<endl;

    //////////////////////////////////////////////////
    // NEIGHBOURING COST PADDING
    /*   float prox=1; //edit as needed
        for (float dx=-prox; dx<=prox;++dx) {
            for (float dy=-prox; dy <=prox ;++dy) {
                if (dx == 0 && dy == 0) continue;  // Skip the current cell

                int neighbor_x = grid_x + dx;
                int neighbor_y = grid_y + dy;
                pair<int, int> neighbor = {neighbor_x, neighbor_y};
               //add new neighbor if not already in the grid
               if(updated_occupancy_grid[{grid_x,grid_y}].visited)
               {
               if
       (updated_occupancy_grid.find(neighbor)==updated_occupancy_grid.end()) {
                   updated_occupancy_grid[neighbor]=CellCost{0.0f, 0.0f, false,
       false};  //set cost as default std::cout<< "Added new neighbor: ("<<
       neighbor.first<< ", "<< neighbor.second<< ")" << std::endl;
               }

            CellCost& neighbor_cell=updated_occupancy_grid[neighbor];


            //calculate proximity cost
            float dist = sqrt(dx *dx + dy*dy);  //grid_resolution
     // euclidean distance between them.
           float proxcost = (proxfactor*2.0f)/(0.1f+dist);

            //update proximity cost only if not already updated
           if (neighbor_cell.proxvisited==false) {
                neighbor_cell.proxcost = proxcost;
                neighbor_cell.cost += neighbor_cell.proxcost; //add proximity
    cost neighbor_cell.proxvisited = true;            //mark it as visited for
    proximity
            }
            }
            }
       }
    }*/

    // Initialize the boundaries to extreme values
    int min_x = INT_MAX;
    int max_x = INT_MIN;
    int min_y = INT_MAX;
    int max_y = INT_MIN;

    for (const auto &cell : updated_occupancy_grid) {
      int xindice = cell.first.first;  // Extract grid x index
      int yindice = cell.first.second; // Extract grid y index

      if (xindice < min_x)
        min_x = xindice;
      if (xindice > max_x)
        max_x = xindice;
      if (yindice < min_y)
        min_y = yindice;
      if (yindice > max_y)
        max_y = yindice;
    }

    gridmap.min_x = min_x;
    gridmap.min_y = min_y;
    gridmap.max_x = max_x;
    gridmap.max_y = max_y;

    // std::cout << "Updated occupancy grid size: " <<
    // updated_occupancy_grid.size() << std::endl;
    /*   for (const auto& [key, value] : updated_occupancy_grid)
       {
           std::cout << "Grid: (" << key.first << ", " << key.second << ") -> "
       << value.cost << std::endl;
       }*/
    gridmap.occupancy_grid = updated_occupancy_grid;
    //  std::cout << "After assignment: Occupancy grid size: " <<
    //  gridmap.occupancy_grid.size() << std::endl;
    // updated_occupancy_grid[current]=CLOSED; //after everything
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
convert_to_pcl(const std::vector<Eigen::Vector3f> &point_vectors) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto &point : point_vectors) {
    cloud->points.emplace_back(point.x(), point.y(), point.z());
  }
  cloud->width = cloud->points.size();
  cloud->height = 1; // Unorganized cloud
  return cloud;
}

void update_rover_pose(Pose &pose, const Vector3f &accel_data,
                       const Vector3f &gyro_data, float delta_time) {
  Vector3f gravity(0.0f, 0.0f, -9.81f);
  Vector3f accel_world =
      pose.orientation * accel_data; // Rotate acceleration into world frame
  accel_world -= gravity;

  pose.velocity += accel_world * delta_time;
  pose.position +=
      pose.velocity * delta_time + 0.5f * accel_world * delta_time * delta_time;
  pose.position /= 1000.0f; // Convert from mm to meters if needed

  Vector3f angular_velocity = gyro_data * delta_time;
  Quaternionf delta_q = Quaternionf(
      AngleAxisf(angular_velocity.norm(), angular_velocity.normalized()));

  pose.orientation = delta_q * pose.orientation;
  pose.orientation.normalize(); // Normalize to prevent drift
}

//helper function to convert rs2 to eigen vector3f
Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec) {
	return Eigen::Vector3f(rs2_vec.x, rs2_vec.y, rs2_vec.z);
}
