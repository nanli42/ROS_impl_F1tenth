#include "f110_opp_detect_node.hpp"

bool requestMap(ros::NodeHandle &nh) {
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;

  while (!ros::service::waitForService("static_map", ros::Duration(3.0)));

  ROS_INFO("Requesting the map...");
  ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");

  if (mapClient.call(req, res)) {
    readMap(res.map);
    return true;
  } else {
    ROS_ERROR("Fail to call map service!");
    return false;
  }
};

void readMap(const nav_msgs::OccupancyGrid& map) {
  loaded_map = map;
  int rows = loaded_map.info.height;
  int cols = loaded_map.info.width;
  double mapResolution = loaded_map.info.resolution;
  ROS_INFO("Received a %d X %d map @ %.3f m/px\n",
    rows,
    cols,
    mapResolution
  );

  // Dynamically resize the grid
  grid.resize(rows);
  for (int i = 0; i < rows; i++) {
    grid[i].resize(cols);
  }
  int currCell = 0;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      if (map.data[currCell] == 0)
        grid[i][j] = true; // free space
      else
        grid[i][j] = false; // occupied
      currCell++;
    }
  }
};
