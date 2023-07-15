#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// define a client type to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PickObjectsNode
{
    private:
        ros::NodeHandle _n;
        MoveBaseClient _move_base_client;

        const double _pick_up_pose[3] = {12.2, 12.3, 1.5707};
        const double _drop_off_pose[3] = {5.0, 8.8, -1.5707};

        void GoToPose(double x, double y, double th)
        {
            // construct goal
            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;

            tf2::Quaternion q;
            q.setRPY(0, 0, th);
            q.normalize();
            goal.target_pose.pose.orientation.x = q.getX();
            goal.target_pose.pose.orientation.y = q.getY();
            goal.target_pose.pose.orientation.z = q.getZ();
            goal.target_pose.pose.orientation.w = q.getW();

            // send goal and wait for completion
            ROS_INFO("Sending goal to (%1.2f, %1.2f, %1.2f)", (float)x, (float)y, (float)th);
            _move_base_client.sendGoal(goal);
            _move_base_client.waitForResult();

            // report success of action
            if(_move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Successfully moved to (%1.2f, %1.2f, %1.2f)", (float)x, (float)y, (float)th);
            } else {
                ROS_INFO("Failed to move to (%1.2f, %1.2f, %1.2f)", (float)x, (float)y, (float)th);
            }
        }

    public:
        PickObjectsNode() :
            _move_base_client("move_base", true)
        {
            // wait 5 sec for move_base action server to come up
            while(!_move_base_client.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            // travel to pick up zone and wait 5 seconds
            ROS_INFO("Traveling to pick-up zone...");
            GoToPose(_pick_up_pose[0], _pick_up_pose[1], _pick_up_pose[2]);
            ROS_INFO("Reached the pick-up zone, waiting for pick-up...");
            ros::Duration(5.0).sleep();

            // travel to drop off zone
            ROS_INFO("Traveling to drop-off zone...");
            GoToPose(_drop_off_pose[0], _drop_off_pose[1], _drop_off_pose[2]);
            ROS_INFO("Arrived at drop-off zone.");
        }
};

int main(int argc, char** argv){
    // initialize ROS node
    ros::init(argc, argv, "pick_objects");

    // create PickObjectsNode object
    PickObjectsNode node;

    return 0;
}