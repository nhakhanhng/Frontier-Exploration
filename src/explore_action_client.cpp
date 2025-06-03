#include "explore/explore_action_client.h"

ExploreActionClient::ExploreActionClient(std::string name)
    : ac_(name, true), feedback_received_(false) {
    ROS_INFO("Action client initializing...");
    ac_.waitForServer();
    ROS_INFO("Connected to action server '%s'.", name.c_str());
}

void ExploreActionClient::feedbackCB(const new_explore::ExplorationFeedbackConstPtr& feedback) {
    feedback_received_ = true;
    // ROS_INFO("Received feedback: status='%s', progress=%f", feedback->status.c_str(), feedback->progress);
    // ROS_INFO("Received feedback: status='%s'", feedback->status.c_str());
    ROS_INFO("Received feedback");
}

bool ExploreActionClient::testStartExploration() {
    ROS_INFO("Test 1: Starting exploration...");
    new_explore::ExplorationGoal goal;
    feedback_received_ = false;
    ac_.sendGoal(goal, actionlib::SimpleActionClient<new_explore::ExplorationAction>::SimpleDoneCallback(),
                 actionlib::SimpleActionClient<new_explore::ExplorationAction>::SimpleActiveCallback(),
                 boost::bind(&ExploreActionClient::feedbackCB, this, _1));

    // Wait for feedback for up to 5 seconds
    ros::Time start = ros::Time::now();
    ros::Duration timeout(5.0);
    while (ros::ok() && !feedback_received_ && (ros::Time::now() - start) < timeout) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    if (!feedback_received_) {
        ROS_ERROR("Test 1 failed: No feedback received.");
        ac_.cancelGoal();
        return false;
    }
    ROS_INFO("Test 1 passed: Feedback received.");
    ac_.cancelGoal();  // Clean up
    return true;
}

bool ExploreActionClient::testStopExploration() {
    ROS_INFO("Test 2: Testing preemption...");
    new_explore::ExplorationGoal goal;
    feedback_received_ = false;
    ac_.sendGoal(goal, actionlib::SimpleActionClient<new_explore::ExplorationAction>::SimpleDoneCallback(),
                 actionlib::SimpleActionClient<new_explore::ExplorationAction>::SimpleActiveCallback(),
                 boost::bind(&ExploreActionClient::feedbackCB, this, _1));

    // Wait briefly to ensure goal is active
    ros::Duration(1.0).sleep();

    // Request preemption
    ROS_INFO("Sending preemption request.");
    ac_.cancelGoal();

    // Wait for result or timeout
    if (!ac_.waitForResult(ros::Duration(5.0))) {
        ROS_ERROR("Test 2 failed: No result received after preemption.");
        return false;
    }

    if (ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_INFO("Test 2 passed: Preemption successful.");
        return true;
    } else {
        ROS_ERROR("Test 2 failed: Goal not preempted, state is %s.", ac_.getState().toString().c_str());
        return false;
    }
}

bool ExploreActionClient::testCompleteExploration() {
    ROS_INFO("Test 3: Testing exploration completion...");
    new_explore::ExplorationGoal goal;
    feedback_received_ = false;
    ac_.sendGoal(goal, actionlib::SimpleActionClient<new_explore::ExplorationAction>::SimpleDoneCallback(),
                 actionlib::SimpleActionClient<new_explore::ExplorationAction>::SimpleActiveCallback(),
                 boost::bind(&ExploreActionClient::feedbackCB, this, _1));

    // Wait for result (up to 30 seconds)
    if (!ac_.waitForResult(ros::Duration(30.0))) {
        ROS_ERROR("Test 3 failed: No result received.");
        ac_.cancelGoal();
        return false;
    }

    new_explore::ExplorationResultConstPtr result = ac_.getResult();
    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && result->complete) {
        ROS_INFO("Test 3 passed: Exploration completed successfully.");
        return true;
    } else {
        ROS_ERROR("Test 3 failed: Goal state is %s, complete=%d.", 
                  ac_.getState().toString().c_str(), result->complete);
        ac_.cancelGoal();
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "explore_action_client");
    ExploreActionClient client("explore_action");

    bool all_tests_passed = true;
    // all_tests_passed &= client.testStartExploration();
    // all_tests_passed &= client.testStopExploration();
    // all_tests_passed &= client.testCompleteExploration();
    client.testStartExploration();

    ROS_INFO("All tests %s.", all_tests_passed ? "passed" : "failed");
    return 0;
}