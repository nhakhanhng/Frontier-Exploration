#include <explore/frontier_search.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <explore/costmap_tools.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/function.hpp>
#include <boost/bind/bind.hpp>
#include <vector> // for vector  
#include <algorithm> // for copy() and assign()  
#include <iterator> // for back_inserter  



namespace frontier_exploration
{
  using costmap_2d::LETHAL_OBSTACLE;
  using costmap_2d::NO_INFORMATION;
  using costmap_2d::FREE_SPACE;

  FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                                double potential_scale, double gain_scale,
                                double min_frontier_size)
    : costmap_(costmap)
    , potential_scale_(potential_scale)
    , gain_scale_(gain_scale)
    , min_frontier_size_(min_frontier_size)
  {
  }

  void HandlePointCloud(std::vector<geometry_msgs::Point32>& RayEndpoints,const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // RayEndpoints_.clear();
              geometry_msgs::Point32 LazerPoint;
              pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
              pcl::fromROSMsg(*msg, pcl_cloud);
              RayEndpoints.clear();
              // // Extract x, y, and z values
              // printf("Call back: %ld\r\n",RayEndpoints.size());
                  // printf("Call back: %x\r\n",&RayEndpoints);
              for (auto point : pcl_cloud.points) {
                  LazerPoint.x = point.x;
                  LazerPoint.y = point.y;
                  LazerPoint.z = point.z;
                  // printf("Adding %f, %f, %f, %ld\r\n",point.x,point.y,point.z,RayEndpoints.size());
                  RayEndpoints.push_back(LazerPoint);
              }
              printf("Call back: %ld\r\n",RayEndpoints.size());
              // RayEndpoints_.push_back(point);
              // printf("\r\n");
  }

  FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,ros::NodeHandle& subscription_nh, double potential_scale,
  double gain_scale, double min_frontier_size)
        : costmap_(costmap)
        , potential_scale_(potential_scale)
        , gain_scale_(gain_scale)
        , min_frontier_size_(min_frontier_size) 
  {
              // geometry_msgs::Point32 LazerPoint;
            // LazerPoint.x = 1;
            // LazerPoint.y = 2.6;
            // LazerPoint.z = 3.78;
            // RayEndpoints_.push_back(LazerPoint);
            // printf("Adding %f, %f, %f,%ld\r\n",LazerPoint.x,LazerPoint.y,LazerPoint.z,RayEndpoints_.size());
            // ROS_DEBUG("Constructor: Vector Address: %p", &RayEndpoints_);
            // boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)> Callback = boost::bind(HandlePointCloud,RayEndpoints_,boost::placeholders::_1);
            // scan_points_sub_ = subscription_nh.subscribe<sensor_msgs::PointCloud2>(
            // "scan_matched_points2", 100,
            // // Callback
            // [this](const sensor_msgs::PointCloud2::ConstPtr& msg) 
            // {
            //       printf("Callback: %p\r\n",&RayEndpoints_);
            //       std::vector<geometry_msgs::Point32> Endpoints;
            //   RayEndpoints_.clear();
            //    geometry_msgs::Point32 LazerPoint;
            //    geometry_msgs::Point32 testPoint;
            //     testPoint.x = 1.0; testPoint.y = 2.0; testPoint.z = 3.0;
            //     RayEndpoints_.push_back(testPoint);
            //   // LazerPoint.x = 1.2;
            //   //     LazerPoint.y = 2.6;
            //   //     LazerPoint.z = 3.0;
            //   //     // printf("Adding %f, %f, %f, %ld\r\n",point.x,point.y,point.z,RayEndpoints_.size());
            //   //     RayEndpoints_.push_back(LazerPoint);
            //   pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            //   pcl::fromROSMsg(*msg, pcl_cloud);
            //   // // Extract x, y, and z values
            //       printf("%ld\r\n",RayEndpoints_.max_size());
            //       printf("Callback: %p\r\n",&RayEndpoints_);
            //         // ROS_DEBUG("Callback: Vector Address: %p, Size before push_back: %lu", &RayEndpoints_, RayEndpoints_.size());
            //   for (auto point : pcl_cloud.points) {
            //       LazerPoint.x = point.x;
            //       LazerPoint.y = point.y;
            //       LazerPoint.z = point.z;
            //       // printf("Adding %f, %f, %f, %ld\r\n",point.x,point.y,point.z,RayEndpoints_.size());
            //       Endpoints.push_back(LazerPoint);
            //   }
              // std::copy(Endpoints.begin(),Endpoints.end(),back_inserter(RayEndpoints_));
              // RayEndpoints_.push_back(point);
              // printf("\r\n");
            // }
            // );
          // printf("%ld\r\n",RayEndpoints_.size());
  } 

  void FrontierSearch::startSubscribe(ros::NodeHandle& subscription_nh) {
    scan_points_sub_ = subscription_nh.subscribe<sensor_msgs::PointCloud2>(
            "scan_matched_points2", 100,
            // Callback
            [this](const sensor_msgs::PointCloud2::ConstPtr& msg) 
            {
                  // printf("Callback: %p\r\n",&RayEndpoints_);
                  // std::vector<geometry_msgs::Point32> Endpoints;
              RayEndpoints_.clear();
              geometry_msgs::Point32 LazerPoint;
              geometry_msgs::Point32 testPoint;
                testPoint.x = 1.0; testPoint.y = 2.0; testPoint.z = 3.0;
                RayEndpoints_.push_back(testPoint);
              // LazerPoint.x = 1.2;
              //     LazerPoint.y = 2.6;
              //     LazerPoint.z = 3.0;
              //     // printf("Adding %f, %f, %f, %ld\r\n",point.x,point.y,point.z,RayEndpoints_.size());
              //     RayEndpoints_.push_back(LazerPoint);
              pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
              pcl::fromROSMsg(*msg, pcl_cloud);
              // // Extract x, y, and z values
                  // printf("%ld\r\n",RayEndpoints_.max_size());
                  printf("Callback: %p\r\n",&RayEndpoints_);
                    // ROS_DEBUG("Callback: Vector Address: %p, Size before push_back: %lu", &RayEndpoints_, RayEndpoints_.size());
              for (auto point : pcl_cloud.points) {
                  LazerPoint.x = point.x;
                  LazerPoint.y = point.y;
                  LazerPoint.z = point.z;
                  // printf("Adding %f, %f, %f, %ld\r\n",point.x,point.y,point.z,RayEndpoints_.size());
                  RayEndpoints_.push_back(LazerPoint);
              }
            }
    );
  }


  std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
  {
    std::vector<Frontier> frontier_list;
  // printf("%ld\r\n",RayEndpoints_.size());
    // Sanity check that robot is inside costmap bounds before searching
    unsigned int mx, my;
    if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
      ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
      return frontier_list;
    }

    // make sure map is consistent and locked for duration of search
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    map_ = costmap_->getCharMap();
    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();

    // initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    // initialize breadth first search
    std::queue<unsigned int> bfs;

    // find closest clear cell to start search
    unsigned int clear, pos = costmap_->getIndex(mx, my);
    if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
      bfs.push(clear);
    } else {
      bfs.push(pos);
      ROS_WARN("Could not find nearby clear cell to start search");
    }

    while (!bfs.empty()) {
      unsigned int idx = bfs.front();
      bfs.pop();

      // iterate over 4-connected neighbourhood
      for (unsigned nbr : nhood4(idx, *costmap_)) {
        // add to queue all free, unvisited cells, use descending search in case
        // initialized on non-free cell
        if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
          visited_flag[nbr] = true;
          bfs.push(nbr);
          // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
          // neighbour)
        } else if (isNewFrontierCell(nbr, frontier_flag)) {
          frontier_flag[nbr] = true;
          Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
          if (new_frontier.size * costmap_->getResolution() >=
              min_frontier_size_) {
            frontier_list.push_back(new_frontier);
          }
        }
      }
    }

    // set costs of frontiers
    for (auto& frontier : frontier_list) {
      frontier.cost = frontierCost(frontier);
    }
    std::sort(
        frontier_list.begin(), frontier_list.end(),
        [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });
    return frontier_list;
  }

  bool FrontierSearch::CanBeFrontier(unsigned int pos) {
    if (map_[pos] == FREE_SPACE) {
        for (unsigned int nbr : nhood4(pos, *costmap_)) {
        if (map_[nbr] == FREE_SPACE) {
          return false;
      }
    }
  }
  if (map_[pos] == NO_INFORMATION) {
        for (unsigned int nbr : nhood4(pos, *costmap_)) {
        if (map_[nbr] == NO_INFORMATION) {
          return false;
      }
    }
  }
  return true;
  }

  void FrontierSearch::search(geometry_msgs::Point position,std::vector<Frontier> &frontier_list) {

    // make sure map is consistent and locked for duration of search
    // printf("Search: %p\r\n",&RayEndpoints_);
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    map_ = costmap_->getCharMap();
    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();

    // initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);
    std::vector<unsigned int> frontier_idxs;
    // initialize breadth first search
    std::queue<unsigned int> bfs;
    // printf("Start\r\n");
    unsigned int mx, my;
      if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        // return frontier_list;
        frontier_list.clear();
      }  
      unsigned int current_pos = costmap_->getIndex(mx, my);

      for (auto old_frontier : frontier_list) {
        for (auto points : old_frontier.points) {
          costmap_->worldToMap(points.x, points.y, mx, my);
          unsigned int pos = costmap_->getIndex(mx, my);
          if (!visited_flag[pos]) {
            bfs.push(pos);
            visited_flag[pos] = true;
          }

        }
      }
      for (auto points : RayEndpoints_) {
          costmap_->worldToMap(points.x, points.y, mx, my);
          unsigned int pos = costmap_->getIndex(mx, my);
          if (!visited_flag[pos]) {
            bfs.push(pos);
            visited_flag[pos] = true;
          }
      }
      frontier_list.clear();
      while (!bfs.empty()) {
        unsigned int idx = bfs.front();
        bfs.pop();
        // printf("idx: %d\r\n",map_[idx]);
        if (isNewFrontierCell(idx,frontier_flag)) {
          frontier_flag[idx] = true;
          // printf("Frontier\r\n");
          frontier_idxs.push_back(idx);
        }
        if (frontier_flag[idx] || CanBeFrontier(idx)) {
            for (auto nbr : nhood8(idx,*costmap_)) {
              if (!visited_flag[nbr] && map_[idx] == costmap_2d::NO_INFORMATION) {
                visited_flag[nbr] = true;
                bfs.push(nbr);
              }
            }
        }
      }
      // printf("Point idx: %d\r\n",frontier_idxs.size());
      frontier_list = buildNewFrontierwithKoraraju(current_pos,frontier_flag,frontier_idxs);
      // set costs of frontiers
      for (auto& frontier : frontier_list) {
        frontier.cost = frontierCost(frontier);
      }
      std::sort(
          frontier_list.begin(), frontier_list.end(),
          [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });
  }

  std::vector<Frontier>  FrontierSearch::buildNewFrontierwithKoraraju( unsigned int reference,std::vector<bool> frontier_flag,
                                                            std::vector<unsigned int> frontier_idxs) {
        std::vector<Frontier> Frontier_list;
        std::vector<bool> visited_flag(size_x_ * size_y_, false);
        std::stack<unsigned int>dfs;
        // cache reference position in world coords
        unsigned int rx, ry;
        double reference_x, reference_y;
        costmap_->indexToCells(reference, rx, ry);
        costmap_->mapToWorld(rx, ry, reference_x, reference_y);
      for (auto idx : frontier_idxs) {
        if (!visited_flag[idx]) {
            visited_flag[idx] = true;
            dfs.push(idx);
            Frontier frontier;
            frontier.min_distance = std::numeric_limits<double>::infinity();
                  unsigned int mx, my;
                  double wx, wy;
                  costmap_->indexToCells(idx, mx, my);
                  costmap_->mapToWorld(mx, my, wx, wy);
                  frontier.initial.x = wx;
                  frontier.initial.y = wy;
                  geometry_msgs::Point point;
                  point.x = wx;
                  point.y = wy;
                  frontier.points.push_back(point);
                  frontier.size = 0;

                  // update frontier size
                  frontier.size++;

                  // update centroid of frontier
                  frontier.centroid.x += wx;
                  frontier.centroid.y += wy;

                  // determine frontier's distance from robot, going by closest gridcell
                  // to robot
                  double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                                        pow((double(reference_y) - double(wy)), 2.0));
                  if (distance < frontier.min_distance) {
                    frontier.min_distance = distance;
                    frontier.middle.x = wx;
                    frontier.middle.y = wy;
                  }         
            while (!dfs.empty()) {
              // printf("DFS empty\r\n");
              unsigned int pos = dfs.top();
              dfs.pop();
              for (auto nbr : nhood8(pos,*costmap_)) {
                if (frontier_flag[nbr] && !visited_flag[nbr]) {
                // printf("neighboor\r\n");
                  costmap_->indexToCells(nbr, mx, my);
                  costmap_->mapToWorld(mx, my, wx, wy);

                  point;
                  point.x = wx;
                  point.y = wy;
                  frontier.points.push_back(point);

                  // update frontier size
                  frontier.size++;

                  // update centroid of frontier
                  frontier.centroid.x += wx;
                  frontier.centroid.y += wy;    

                  // determine frontier's distance from robot, going by closest gridcell
                  // to robot
                  distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                                        pow((double(reference_y) - double(wy)), 2.0));
                  if (distance < frontier.min_distance) {
                    frontier.min_distance = distance;
                    frontier.middle.x = wx;
                    frontier.middle.y = wy;
                  }
                  dfs.push(nbr);
                  visited_flag[nbr] = true;
                }
              }
            }
            // printf("Distance: %2f\r\n",frontier.min_distance);
            // printf("Size: %d\r\n",frontier.size);
            frontier.centroid.x /= frontier.size;
            frontier.centroid.y /= frontier.size;
            if (frontier.size > 3) {

            Frontier_list.push_back(frontier);
            }
        }
      }
    return Frontier_list;
  }

  Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                            unsigned int reference,
                                            std::vector<bool>& frontier_flag)
  {
    // initialize frontier structure
    Frontier output;
    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    // record initial contact point for frontier
    unsigned int ix, iy;
    costmap_->indexToCells(initial_cell, ix, iy);
    costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

    // push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    // cache reference position in world coords
    unsigned int rx, ry;
    double reference_x, reference_y;
    costmap_->indexToCells(reference, rx, ry);
    costmap_->mapToWorld(rx, ry, reference_x, reference_y);

    while (!bfs.empty()) {
      unsigned int idx = bfs.front();
      bfs.pop();

      // try adding cells in 8-connected neighborhood to frontier
      for (unsigned int nbr : nhood8(idx, *costmap_)) {
        // check if neighbour is a potential frontier cell
        if (isNewFrontierCell(nbr, frontier_flag)) {
          // mark cell as frontier
          frontier_flag[nbr] = true;
          unsigned int mx, my;
          double wx, wy;
          costmap_->indexToCells(nbr, mx, my);
          costmap_->mapToWorld(mx, my, wx, wy);

          geometry_msgs::Point point;
          point.x = wx;
          point.y = wy;
          output.points.push_back(point);

          // update frontier size
          output.size++;

          // update centroid of frontier
          output.centroid.x += wx;
          output.centroid.y += wy;

          // determine frontier's distance from robot, going by closest gridcell
          // to robot
          double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                                pow((double(reference_y) - double(wy)), 2.0));
          if (distance < output.min_distance) {
            output.min_distance = distance;
            output.middle.x = wx;
            output.middle.y = wy;
          }

          // add to queue for breadth first search
          bfs.push(nbr);
        }
      }
    }

    // average out frontier centroid
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
    return output;
  }

  bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                        const std::vector<bool>& frontier_flag)
  {
    // check that cell is unknown and not already marked as frontier
    if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
      return false;
    }

    // frontier cells should have at least one cell in 4-connected neighbourhood
    // that is free
    for (unsigned int nbr : nhood4(idx, *costmap_)) {
      // if (map_[nbr] == LETHAL_OBSTACLE ) return false;
      if (map_[nbr] == FREE_SPACE) {
        return true;
      }
    }

    return false;
  }

  double FrontierSearch::frontierCost(const Frontier& frontier)
  {
    return (potential_scale_ * frontier.min_distance *
            costmap_->getResolution()) -
          (gain_scale_ * frontier.size * costmap_->getResolution());
  }
}