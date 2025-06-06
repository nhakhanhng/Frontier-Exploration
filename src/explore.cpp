/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.1;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = timeout;
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),relative_nh_,
                                                 potential_scale_, gain_scale_,orientation_scale_,
                                                 min_frontier_size);

          // printf("Construct: %p\r\n",&search_.RayEndpoints_);
  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");
  frontiers.reserve(10);
  search_.startSubscribe(relative_nh_);
  last_progress_ = ros::Time::now().sec;
  // ROS_DEBUG("Sended Goal");
  start_position_ = costmap_client_.getRobotPose().position;
  // move_base_msgs::MoveBaseGoal rotate_goal;
  //   rotate_goal.target_pose.pose.position = start_position_;
  //   rotate_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
  //   rotate_goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  //   rotate_goal.target_pose.header.stamp = ros::Time::now();
  //   move_base_client_.sendGoal(rotate_goal);
  //   ros::Duration(2).sleep(); // wait for half rotation
  //   rotate_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(2 * M_PI);
  //   move_base_client_.sendGoal(rotate_goal);
  //   ros::Duration(2).sleep(); // wait for the other half rotation
    // move_base_client_.waitForResult();
  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) {
  makePlan(); });
  exploring_timer_.stop();
  // filter_frontier_timer_ =
  //     relative_nh_.createTimer(ros::Duration(1. / filter_frontier_frequency_),
  //                              [this](const ros::TimerEvent&) {
  // filterFrontiers(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0.8;
  blue.g = 0;
  blue.b = 0.8;
  blue.a = 1;
  std_msgs::ColorRGBA red;
  red.r = 0.8;
  red.g = 0;
  red.b = 0;
  red.a = 1;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 0.8;
  green.b = 0;
  green.a = 1;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.8 / frontier.cost), 0.2);
    // double scale = 2;
    // printf("Scale: %2f\r\n",scale);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  search_.search(pose.position,frontiers);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }
  if (frontiers.empty()) {
    ROS_DEBUG("No frontiers found, rotating in place.");
    
    // return;
    frontiers = search_.searchFrom(pose.position);
  }

  if (frontiers.empty()) {
    ROS_DEBUG("No frontiers found, stopping exploration.");
    stop();
    is_explore_completed_ = true;
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    ROS_DEBUG("Visualizing frontiers.");
    visualizeFrontiers(frontiers);
  }

  actionlib::SimpleClientGoalState state = move_base_client_.getState();
  ROS_DEBUG("Current state: %s", state.toString().c_str());
  if (state == actionlib::SimpleClientGoalState::ACTIVE) {
    if (ros::Time::now().sec - last_progress_ > progress_timeout_) {
      ROS_WARN("Goal is active but no progress for a long time, canceling goal");
      move_base_client_.cancelAllGoals();
      ROS_DEBUG("After timeout state: %s", move_base_client_.getState().toString().c_str());
      frontier_blacklist_.push_back(prev_goal_);
      frontiers.erase(std::remove_if(frontiers.begin(), frontiers.end(),
                 [this](const frontier_exploration::Frontier& f) {
                   return f.centroid == prev_goal_;
                 }),
              frontiers.end());
      // makePlan();
      // return;
    } else {
      return;
    }
  }

  // find non blacklisted frontier
  ROS_DEBUG("Finding non-blacklisted frontier.");
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    stop();
    return;
  }

  

  geometry_msgs::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  // bool same_goal = target_position == prev_goal_;
  prev_goal_ = target_position;
  // if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now().sec;
    prev_distance_ = frontier->min_distance;
  // }
  // black list if we've made no progress for a long time
  // if (ros::Time::now() - last_progress_ > progress_timeout_) {
  //   frontier_blacklist_.push_back(target_position);
  //   ROS_DEBUG("Adding current goal to black list");
  //   makePlan();
  //   return;
  // }

  // we don't need to do anything if we still pursuing the same goal
  // if (same_goal) {
  //   printf("Same goal\r\n");
  //   return;
  // }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  ROS_DEBUG("Sended Goal");
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}


// void filterFrontiers() {
//   std::vector<frontier_exploration::Frontier> new_frontiers;
//   for (const auto& frontier : frontiers) {
//     if (!goalOnBlacklist(frontier.centroid) && !isOldGoal(frontier.centroid,prev_goal_)) {
//       new_frontiers.push_back(frontier);
//     }
//   }
//   frontiers = std::move(new_frontiers);
// }

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 20;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

bool Explore::isOldGoal(const geometry_msgs::Point& new_goal,const geometry_msgs::Point& prev_goal) {
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  double x_diff = fabs(new_goal.x - prev_goal.x);
  double y_diff = fabs(new_goal.y - prev_goal.y);

  if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;

  return false;

}


void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    
    ROS_DEBUG("Adding current goal to black list");
  }

  else if (status == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_DEBUG("Goal was preempted, not adding to black list");
    stop();
    return;
  }
  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { this->makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

bool Explore::isExploreCompleted()
{
  return is_explore_completed_;
}
void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  // is_explore_completed_ = true;
  // move_base_msgs::MoveBaseGoal goal;
  // goal.target_pose.pose.position = start_position_;
  // goal.target_pose.pose.orientation.w = 1.0;
  // goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  // goal.target_pose.header.stamp = ros::Time::now();
  // move_base_client_.sendGoal(goal);
  // move_base_client_.waitForResult();
  ROS_INFO("Exploration stopped.");
}

}  // namespace explore

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "explore");
//   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
//                                      ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//   }
//   explore::Explore explore;
//   ros::spin();

//   return 0;
// }
