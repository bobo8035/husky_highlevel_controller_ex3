#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
	if (!readParameters()) 
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

	// subscribers
	scan_sub_ = nodeHandle_.subscribe(subscriberTopic_, queue_size , &HuskyHighlevelController::scanCallback, this);
    // publishers
    cmd_pub_  = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel",100);
    vis_pub_  = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker",10);
	
    ROS_INFO("Successfully launched node.");
}

bool HuskyHighlevelController::readParameters()
    {
        if (!nodeHandle_.getParam("scan_sub_topic", subscriberTopic_)) return false;

        if (!nodeHandle_.getParam("scan_sub_queue_size", queue_size))return false;

        return true;
    }

HuskyHighlevelController::~HuskyHighlevelController()
{
}

void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan &scan_msg)
{
    float smallest_distance = 100;
    // the angle corresponding to the minimum distance


    //number of the elements in ranges array
    int arr_size = floor((scan_msg.angle_max-scan_msg.angle_min)/scan_msg.angle_increment);
    for (int i=0 ; i< arr_size ;i++)
    {
        if (scan_msg.ranges[i] < smallest_distance)
        {
            smallest_distance = scan_msg.ranges[i];
            alpha_pillar = (scan_msg.angle_min + i*scan_msg.angle_increment);

        }
    }
    //Pillar Husky offset pose 
    x_pillar = smallest_distance*cos(alpha_pillar);
    y_pillar = smallest_distance*sin(alpha_pillar);
    //ROS_INFO_STREAM("ROS_INFO_STREAM Minimum laser distance(m): "<<smallest_distance);
    ROS_INFO("Pillar offset angle(rad):%lf", alpha_pillar);
	ROS_INFO("pillar x distance(m):%lf", x_pillar);
	ROS_INFO("pillar y distance(m):%lf", y_pillar);

    //P-Controller to drive husky towards the pillar
    HuskyHighlevelController::pController();
    HuskyHighlevelController::visMsg();
    cmd_pub_.publish(vel_msg_);
    vis_pub_.publish( marker );
}

    void HuskyHighlevelController::pController()
    {
        //propotinal gain
        float p_gain_vel = 0.2;
        float p_gain_ang = 0.4;
        if (x_pillar > 0.2)
        {
            if (x_pillar <= 0.5 )
            {
                vel_msg_.linear.x = 0;
                vel_msg_.linear.y = 0; 
                vel_msg_.angular.z =0;

            }
            else 
            {
                vel_msg_.linear.x = x_pillar * p_gain_vel  ;
                vel_msg_.linear.y = y_pillar * p_gain_vel ; 
                vel_msg_.angular.z = -(alpha_pillar * p_gain_ang ) ;

            }
 
       }
       else
       {
            vel_msg_.linear.x = 0;
            vel_msg_.linear.y = 0; 
            vel_msg_.angular.z =0;
       }

        
    }
    void HuskyHighlevelController::visMsg()
    {
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "pillar";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x_pillar;
        marker.pose.position.y = y_pillar; 
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }


} /* namespace */
