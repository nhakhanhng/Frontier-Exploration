
#include "explore/explore_action_server.h"

ExploreActionServer::ExploreActionServer(std::string name)
    : as_(nh_, name, boost::bind(&ExploreActionServer::executeCB, this, _1), false),
      action_name_(name), explore_() {
    as_.start();
    ROS_INFO("Action server '%s' started.", name.c_str());
}

void ExploreActionServer::executeCB(const new_explore::ExplorationGoalConstPtr &goal) {
    ROS_INFO("Starting exploration process...");

    ros::Rate rate(1);  // 1 Hz frequency
    while (ros::ok() && !explore_.isExploreCompleted()) {
        // Check for preemption request
        if (as_.isPreemptRequested()) {
            ROS_INFO("Received request to stop exploration.");
            stop_exploration();
            as_.setPreempted();
            return;
        }

        // Publish feedback
        new_explore::ExplorationFeedback feedback;
        feedback.status = "exploring";
        // feedback.progress = 0.5;  // Placeholder, replace with actual progress
        as_.publishFeedback(feedback);

        rate.sleep();
    }

    // When exploration completes
    if (ros::ok()) {
        new_explore::ExplorationResult result;
        result.complete = true;
        ROS_INFO("Exploration completed.");
        as_.setSucceeded(result);
    }
}

bool ExploreActionServer::is_exploration_complete() {
    // Logic to check if exploration is complete
    // Example: check a topic or service
    return false;  // Replace with actual logic
}

void ExploreActionServer::stop_exploration() {
    // Logic to stop explore_lite
    // Example: call a service or send a signal
    ROS_INFO("Exploration stopped.");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "explore_action_server");
    ROS_INFO("Initializing Explore Action Server...");

    ExploreActionServer explore_action_server("explore_action");
    ros::spin();

    return 0;
}
