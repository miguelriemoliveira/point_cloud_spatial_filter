#ifndef _PERCEPTION_PREPROCESSING_NODE_
#define _PERCEPTION_PREPROCESSING_NODE_
#endif

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

#include <point_cloud_spatial_filter/point_cloud_spatial_filter.h>

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */

int main (int argc, char** argv)
{
	ros::init(argc, argv, "point_cloud_spatial_filter_node"); // Initialize ROS coms
	ros::NodeHandle* n;
	n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	PerceptionPreprocessing pp(n); //initialize the class

	ros::Time t = ros::Time::now();
	ros::Rate loop_rate(30);
	ros::spin();

	return 1;
}

