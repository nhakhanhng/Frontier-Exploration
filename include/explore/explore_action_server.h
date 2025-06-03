
#ifndef EXPLORE_ACTION_SERVER_H
#define EXPLORE_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <new_explore/ExplorationAction.h>  // Replace 'your_package' with your actual package name
#include "explore.h"

class ExploreActionServer {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<new_explore::ExplorationAction> as_;
    explore::Explore explore_;
    std::string action_name_;

public:
    ExploreActionServer(std::string name);
    void executeCB(const new_explore::ExplorationGoalConstPtr &goal);
    bool is_exploration_complete();
    void stop_exploration();
};

#endif  // EXPLORE_ACTION_SERVER_H
