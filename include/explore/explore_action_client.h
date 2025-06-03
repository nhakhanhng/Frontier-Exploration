#ifndef EXPLORE_ACTION_CLIENT_H
#define EXPLORE_ACTION_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <new_explore/ExplorationAction.h>

class ExploreActionClient {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<new_explore::ExplorationAction> ac_;
    bool feedback_received_;

    void feedbackCB(const new_explore::ExplorationFeedbackConstPtr& feedback);

public:
    ExploreActionClient(std::string name);
    bool testStartExploration();
    bool testStopExploration();
    bool testCompleteExploration();
};

#endif  // EXPLORE_ACTION_CLIENT_H