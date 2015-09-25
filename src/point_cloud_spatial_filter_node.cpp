#ifndef _PERCEPTION_PREPROCESSING_NODE_
#define _PERCEPTION_PREPROCESSING_NODE_
#endif

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

#include <point_cloud_spatial_filter/point_cloud_spatial_filter.h>

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */


void callbackDynamicReconfigure(point_cloud_spatial_filter::DynamicConfigurationConfig &config, uint32_t level, PerceptionPreprocessing pp) 
{
// Callback to allow Dynamic Reconfiguration of several parameters

    ROS_INFO("Reconfigure Request: Setting x min to %f",  config.x_min);
    ROS_INFO("Reconfigure Request: Setting x max to %f",  config.x_max);
    ROS_INFO("Reconfigure Request: Setting y min to %f",  config.y_min);
    ROS_INFO("Reconfigure Request: Setting y max to %f",  config.y_max);
    ROS_INFO("Reconfigure Request: Setting z min to %f",  config.z_min);
    ROS_INFO("Reconfigure Request: Setting z max to %f",  config.z_max);

    pp.x_min = (double) config.x_min;
    pp.x_max = (double) config.x_max;
    pp.y_min = (double) config.x_min;
    pp.y_max = (double) config.x_max;
    pp.z_min = (double) config.x_min;
    pp.z_max = (double) config.x_max;
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "point_cloud_spatial_filter_node"); // Initialize ROS coms

	ros::NodeHandle* n;
	n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle
	
	PerceptionPreprocessing pp(n); //initialize the class

     // Setup the dynamic online configuration
    dynamic_reconfigure::Server<point_cloud_spatial_filter::DynamicConfigurationConfig> server;
    dynamic_reconfigure::Server<point_cloud_spatial_filter::DynamicConfigurationConfig>::CallbackType f;
    f = boost::bind(&callbackDynamicReconfigure, _1, _2, pp);
    server.setCallback(f);


	ros::Time t = ros::Time::now();
	ros::Rate loop_rate(30);
	ros::spin();

	return 1;
}