#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
    // Read ros param, return true if successful
	bool readParameters();

	// Callback methods
	void scanCallback(const sensor_msgs::LaserScan &scan_msg);

	//P-controller method
	void pController();
	// a Method to create marker visualization msg
	void visMsg();

	// node handle
	ros::NodeHandle nodeHandle_;
	
	// subscriber to /scan topic
	ros::Subscriber  scan_sub_;

	std::string subscriberTopic_;
	int queue_size;

	//------Pillar info----
	////pillar position
	float x_pillar;
	float y_pillar;
	// the orientation of the pillar with respect to the x_axis
	float alpha_pillar;

	//-----Publishers-----////publisher to /cmd_vel
	ros::Publisher cmd_pub_;
	// publisher to /visualization_marker
	ros::Publisher vis_pub_;
	//------msgs-------////twist msg
	geometry_msgs::Twist vel_msg_;
	visualization_msgs::Marker marker;

};

} /* namespace */
