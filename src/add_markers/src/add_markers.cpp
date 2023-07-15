#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <math.h>

enum RobotState
{
    MOVING_TO_PICK_UP,
    MOVING_TO_DROP_OFF,
    STOPPED
};

class AddMarkersNode
{
    private:
        ros::NodeHandle _n;
        ros::Publisher _marker_publisher;
        ros::Subscriber _pose_subscriber;
        
        RobotState _robot_state;
        const double _pick_up_pose[3] = {12.2, 12.3, 1.5707};
        const double _drop_off_pose[3] = {5.0, 8.8, -1.5707};
        const double _distance_thresh = 0.5;

        void PublishShape(bool create, double x, double y)
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();

            marker.ns = "virtual_object";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = (create ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE);

            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;

            marker.scale.x = 0.25;
            marker.scale.y = 0.25;
            marker.scale.z = 0.25;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            _marker_publisher.publish(marker);
        }

    public:
        AddMarkersNode()
        {
            _marker_publisher = _n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
            _pose_subscriber = _n.subscribe("/amcl_pose", 10, &AddMarkersNode::processPoseCallback, this);

            _robot_state = MOVING_TO_PICK_UP;

            // sleep to ensure rviz is ready for commands
            ros::Duration(2.0).sleep();

            ROS_INFO("Adding initial object at pick up pose.");
            PublishShape(true, _pick_up_pose[0], _pick_up_pose[1]);
        }

        void processPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
        {
            using namespace std;

            double x = msg.pose.pose.position.x;
            double y = msg.pose.pose.position.y;

            if (_robot_state == MOVING_TO_PICK_UP)
            {
                double dist_from_pick_up = std::sqrt(
                    std::pow(x - _pick_up_pose[0], 2) + 
                    std::pow(y - _pick_up_pose[1], 2));

                if (dist_from_pick_up < _distance_thresh)
                {
                    // delete pick up shape and change node state
                    ROS_INFO("Reached pick-up position.");
                    PublishShape(false, _pick_up_pose[0], _pick_up_pose[1]);
                    _robot_state = MOVING_TO_DROP_OFF;
                }
            } 
            else if (_robot_state == MOVING_TO_DROP_OFF)
            {
                double dist_from_drop_off = std::sqrt(
                    std::pow(x - _drop_off_pose[0], 2) + 
                    std::pow(y - _drop_off_pose[1], 2));

                if (dist_from_drop_off < _distance_thresh)
                {
                    // show object at drop off position
                    ROS_INFO("Reached drop-off position.");
                    PublishShape(true, _drop_off_pose[0], _drop_off_pose[1]);
                    _robot_state = MOVING_TO_DROP_OFF;
                }
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_markers");
    
    AddMarkersNode node;

    ros::spin();

    return 0;
}