#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tue_move_base_msgs/MoveBaseActionGoal.h>

ros::Publisher pub_initial_pose_;

void callbackMoveBase(const tue_move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {
    geometry_msgs::PoseStamped goal = msg->goal.path.back();
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header = goal.header;
    init_pose.pose.pose.position = goal.pose.position;
    init_pose.pose.pose.orientation = goal.pose.orientation;
    for(unsigned int i = 0; i < 36; ++i) {
        init_pose.pose.covariance[i] = 0;
    }
    pub_initial_pose_.publish(init_pose);
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "amigo_teleporter");
    ros::NodeHandle nh;

    pub_initial_pose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

    ros::Subscriber sub_move_base_goal = nh.subscribe("/move_base/goal", 10, &callbackMoveBase);

    ros::spin();
}
