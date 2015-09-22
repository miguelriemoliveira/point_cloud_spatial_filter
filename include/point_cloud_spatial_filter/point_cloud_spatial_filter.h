#ifndef _PERCEPTION_PREPROCESSING_H_
#define _PERCEPTION_PREPROCESSING_H_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <boost/bind.hpp>

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <interactive_markers/menu_handler.h>
#include <ros/package.h>
#include <rospack/rospack.h>

//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


/* _________________________________
   |                                 |
   |           Namespaces            |
   |_________________________________| */
using namespace std;
using namespace pcl;
using namespace ros;
using namespace visualization_msgs;


/* _________________________________
   |                                 |
   |        Class definition         |
   |_________________________________| */

class PerceptionPreprocessing
{
    public:
        //Type defs
        typedef PointXYZRGB T;

        //local variables
        string _name;
        bool _verb;
        ros::NodeHandle* _p_nh; // The pointer to the node handle
        ros::NodeHandle _nh; // The node handle
        ros::NodeHandle _priv_nh; // The node handle
        ros::NodeHandle* _p_priv_nh; // The node handle
        boost::shared_ptr<Subscriber> _p_pcin_subscriber;
        boost::shared_ptr<Publisher> _p_pc_publisher;
        boost::shared_ptr<Publisher> _p_marker_publisher;
        boost::shared_ptr<tf::TransformListener> _p_transform_listener;


        boost::shared_ptr<PointCloud<T> > pc_in;
        boost::shared_ptr<PointCloud<T> > pc_transformed;
        boost::shared_ptr<PointCloud<T> > pc_filtered;
        boost::shared_ptr<PointCloud<T> > pc_downsampled;
        sensor_msgs::PointCloud2 msg_out;
        boost::shared_ptr<ConditionAnd<T> > _range_cond;
        boost::shared_ptr<ConditionAnd<T> > _range_cond1;

        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
        interactive_markers::MenuHandler menu_handler;

        /* _________________________________
           |                                 |
           |           PARAMETERS			|
           |_________________________________| */
        double _x_min, _x_max, _y_min, _y_max, _z_min, _z_max;
        bool _voxelize;
        double _x_voxel, _y_voxel, _z_voxel;
        bool _flg_configure;
        std::string _point_cloud_in;

        string _fixed_frame_id;
        /* _________________________________
           |                                 |
           |           CONSTRUCTORS          |
           |_________________________________| */

        PerceptionPreprocessing(){};

        PerceptionPreprocessing(ros::NodeHandle* n)
        {
            _p_nh = n; //if this is a node set both the nodehandle and private node handle to n
            _p_priv_nh = n; 
            onInit();
        };

        /**
         * @brief Destructor
         */
        ~PerceptionPreprocessing()
        {
            ROS_INFO("%s: Destructor called", _name.c_str());
            ROS_INFO("%s: Finished destructor", _name.c_str());
        };


        /* _________________________________
           |                                 |
           |           CLASS METHODS         |
           |_________________________________| */

        void onInit(void)
        {

            //initialize parameters
            _name = _p_priv_nh->getNamespace();

            _p_priv_nh->param<double>("x_max", _x_max , 2.0);
            _p_priv_nh->param<double>("x_min", _x_min , -2.0);
            _p_priv_nh->param<double>("y_max", _y_max , 2.0);
            _p_priv_nh->param<double>("y_min", _y_min , -2.0);
            _p_priv_nh->param<double>("z_max", _z_max , 3.0);
            _p_priv_nh->param<double>("z_min", _z_min , 0.0);
            _p_priv_nh->param<bool>("voxelize", _voxelize , false);
            _p_priv_nh->param<double>("x_voxel", _x_voxel , 0.01);
            _p_priv_nh->param<double>("y_voxel", _y_voxel , 0.01);
            _p_priv_nh->param<double>("z_voxel", _z_voxel , 0.01);
            _p_priv_nh->param<bool>("configure", _flg_configure , false);
            _p_priv_nh->param<std::string>("point_cloud_in", _point_cloud_in , "camera_default/depth_registered/points");
            ROS_INFO_STREAM("_point_cloud_in topic name is " << _point_cloud_in);

            _p_priv_nh->param<std::string>("fixed_frame_id", _fixed_frame_id, "/camera_default_rgb_optical_frame");
            ROS_INFO_STREAM("_fixed_frame_id is " << _fixed_frame_id); 

            //create point clouds
            pc_in = (boost::shared_ptr<PointCloud<T> >) new PointCloud<T>;
            pc_transformed = (boost::shared_ptr<PointCloud<T> >) new PointCloud<T>;
            pc_filtered = (boost::shared_ptr<PointCloud<T> >) new PointCloud<T>;
            pc_downsampled = (boost::shared_ptr<PointCloud<T> >) new PointCloud<T>;

            //initialize condition filter
            setConditionFilter();

            _p_transform_listener = (boost::shared_ptr<tf::TransformListener>) new tf::TransformListener;

            //initialize the subscriber
            _p_pcin_subscriber = (boost::shared_ptr<Subscriber>) new Subscriber;
            *_p_pcin_subscriber = _p_nh->subscribe (_point_cloud_in, 1, &PerceptionPreprocessing::pointCloudCallback, this);

            //initialize the publisher
            _p_pc_publisher = (boost::shared_ptr<Publisher>) new Publisher;
            *_p_pc_publisher = _p_priv_nh->advertise<sensor_msgs::PointCloud2>("points", 100);


            //Initialize the interactive markers
            if (_flg_configure==true)
            {
                //initialize the publisher
                _p_marker_publisher = (boost::shared_ptr<Publisher>) new Publisher;
                *_p_marker_publisher = _p_priv_nh->advertise<visualization_msgs::MarkerArray>("rviz/filter", 100);

                server.reset( new interactive_markers::InteractiveMarkerServer("preprocessing_box","",false) );
                make6DofMarker( "preprocessing_box", "", tf::Vector3(_x_min, _y_min, _z_min), tf::Vector3(_x_max, _y_max, _z_max));


                ROS_INFO_STREAM("Configuration mode is on");
            }
            else
            {
                ROS_INFO_STREAM("Configuration mode is off");
            }

        };

        void publishVisualizationMarkers(void)
        {
            visualization_msgs::MarkerArray marker_array; 
            geometry_msgs::Point p;

            /* _________________________________
               |                                 |
               |             DRAW WIREFRAME      |
               |_________________________________| */
            if (1)
            {
                visualization_msgs::Marker marker;
                //marker.header.frame_id = "/camera_rgb_optical_frame";
                marker.header.frame_id = _fixed_frame_id;
                marker.ns = "wireframe";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::LINE_LIST;
                marker.action = visualization_msgs::Marker::ADD;
                marker.lifetime = Duration(0);

                marker.pose.position.x = (_x_min + _x_max)/2;
                marker.pose.position.y = (_y_min + _y_max)/2;
                marker.pose.position.z = (_z_min + _z_max)/2;

                marker.scale.x = 0.01; //width of the line

                double x = fabs(_x_min - marker.pose.position.x); 
                double y = fabs(_y_min - marker.pose.position.y); 
                double z = fabs(_z_min - marker.pose.position.z); 

                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 0.7;
                marker.color.a = 1.0;

                p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
                p.x = -x; p.y =  y; p.z =  z; marker.points.push_back(p);
                p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
                p.x = -x; p.y =  y; p.z = -z; marker.points.push_back(p);
                p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
                p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
                p.x = -x; p.y =  y; p.z = -z; marker.points.push_back(p);
                p.x = -x; p.y =  y; p.z =  z; marker.points.push_back(p);

                p.x =  x; p.y =  -y; p.z =  z; marker.points.push_back(p);
                p.x = -x; p.y =  -y; p.z =  z; marker.points.push_back(p);
                p.x =  x; p.y =  -y; p.z = -z; marker.points.push_back(p);
                p.x = -x; p.y =  -y; p.z = -z; marker.points.push_back(p);
                p.x =  x; p.y =  -y; p.z = -z; marker.points.push_back(p);
                p.x =  x; p.y =  -y; p.z =  z; marker.points.push_back(p);
                p.x = -x; p.y =  -y; p.z = -z; marker.points.push_back(p);
                p.x = -x; p.y =  -y; p.z =  z; marker.points.push_back(p);

                p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
                p.x =  x; p.y = -y; p.z =  z; marker.points.push_back(p);
                p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
                p.x =  x; p.y = -y; p.z = -z; marker.points.push_back(p);
                p.x = -x; p.y =  y; p.z =  z; marker.points.push_back(p);
                p.x = -x; p.y = -y; p.z =  z; marker.points.push_back(p);
                p.x = -x; p.y =  y; p.z = -z; marker.points.push_back(p);
                p.x = -x; p.y = -y; p.z = -z; marker.points.push_back(p);


                marker_array.markers.push_back(marker);
            }
            _p_marker_publisher->publish(marker_array);
        }

        Marker makeBox( InteractiveMarker &msg )
        {
            Marker marker;

            marker.type = Marker::SPHERE;
            marker.scale.x = msg.scale * 0.045;
            marker.scale.y = msg.scale * 0.045;
            marker.scale.z = msg.scale * 0.045;
            //marker.scale.x = msg.scale * 0.45;
            //marker.scale.y = msg.scale * 0.45;
            //marker.scale.z = msg.scale * 0.45;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.color.a = 1.0;

            return marker;
        }

        InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
        {
            InteractiveMarkerControl control;
            control.always_visible = true;
            control.markers.push_back( makeBox(msg) );
            msg.controls.push_back( control );

            return msg.controls.back();
        }

        void make6DofMarker(string name, string description, const tf::Vector3& position1, const tf::Vector3& position2)
        {
            InteractiveMarker int_marker;
            //int_marker.header.frame_id = "/camera_rgb_optical_frame";
            int_marker.header.frame_id = _fixed_frame_id;
            tf::pointTFToMsg(position1, int_marker.pose.position);
            int_marker.scale = 0.5;

            int_marker.name = "corner 1";
            int_marker.description = description;

            // insert a box
            makeBoxControl(int_marker);
            int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

            InteractiveMarkerControl control;

            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "move_x_1";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "move_z_1";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "move_y_1";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);

            server->insert(int_marker);
            server->setCallback(int_marker.name, boost::bind(&PerceptionPreprocessing::processFeedback, this, _1));
            // insert a box
            tf::pointTFToMsg(position2, int_marker.pose.position);
            makeBoxControl(int_marker);

            int_marker.name = "corner 2";
            int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls[4].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "move_x_2";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "move_z_2";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "move_y_2";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            server->insert(int_marker);
            server->setCallback(int_marker.name, boost::bind(&PerceptionPreprocessing::processFeedback, this, _1));

            //Create the menu marker
            menu_handler.insert( "Save box to configuration file", boost::bind(&PerceptionPreprocessing::processFeedback, this, _1) );

            //menu_handler.insert( "Select frame_id", boost::bind(&PerceptionPreprocessing::processFeedback, this, _1) );
            //interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Select frame id" );
            //menu_handler.insert( sub_menu_handle, "/camera_rgb_optical_frame", boost::bind(&PerceptionPreprocessing::processFeedback, this, _1));
            //menu_handler.insert( sub_menu_handle, "/base_link", boost::bind(&PerceptionPreprocessing::processFeedback, this, _1));

            tf::Vector3 position0(0,0,2);
            tf::pointTFToMsg(position0, int_marker.pose.position);
            makeMenuMarker( position1 );

            server->applyChanges();
        }

        void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
        {
            ROS_INFO_STREAM("Feedback from marker " << feedback->marker_name << " control" << feedback->control_name);

            switch ( feedback->event_type )
            {
                case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
                    if (feedback->marker_name=="corner 1")
                    {
                        _x_min = feedback->pose.position.x;
                        _y_min = feedback->pose.position.y;
                        _z_min = feedback->pose.position.z;
                    }
                    else if (feedback->marker_name=="corner 2")
                    {
                        _x_max = feedback->pose.position.x;
                        _y_max = feedback->pose.position.y;
                        _z_max = feedback->pose.position.z;
                    }
                    setConditionFilter();

                    ROS_INFO_STREAM("x_min = " << _x_min << " y_min = " << _y_min << " z_min = " << _z_min);
                    ROS_INFO_STREAM("x_max = " << _x_max << " y_max = " << _y_max << " z_max = " << _z_max);
                    break;
                case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
                    ROS_INFO_STREAM(": menu item " << feedback->menu_entry_id << " clicked");
                    switch (feedback->menu_entry_id)
                case 1:
                        ROS_WARN("Dumping current configuration to file");
                        string path = ros::package::getPath("point_cloud_spatial_filter");

                        _p_priv_nh->setParam("x_min", _x_min);
                        _p_priv_nh->setParam("y_min", _y_min);
                        _p_priv_nh->setParam("z_min", _z_min);
                        _p_priv_nh->setParam("x_max", _x_max);
                        _p_priv_nh->setParam("y_max", _y_max);
                        _p_priv_nh->setParam("z_max", _z_max);
                        std::string cmd = "rosparam dump " + path + "/params/" + "default_params.yaml /point_cloud_filter -v";
                        system(cmd.c_str());

                        //call(["rosparam dump three_points.yaml /ur5_with_asus_calibration -v",""], shell=True)
                        break;
                Default:
                        ROS_WARN("Unknonw menu entry id");
                        break;

                    break;
            }

            server->applyChanges();
        }

        void setConditionFilter(void)
        {
            _range_cond.reset();	
            _range_cond = (boost::shared_ptr<ConditionAnd<T> >) new ConditionAnd<T>();
            _range_cond->addComparison (boost::shared_ptr< const FieldComparison<T> > (new FieldComparison<T> ("x", ComparisonOps::GT, _x_min)));
            _range_cond->addComparison (boost::shared_ptr< const FieldComparison<T> > (new FieldComparison<T> ("x", ComparisonOps::LT, _x_max)));
            _range_cond->addComparison (boost::shared_ptr< const FieldComparison<T> > (new FieldComparison<T> ("y", ComparisonOps::GT, _y_min)));
            _range_cond->addComparison (boost::shared_ptr< const FieldComparison<T> > (new FieldComparison<T> ("y", ComparisonOps::LT, _y_max)));
            _range_cond->addComparison (boost::shared_ptr< const FieldComparison<T> > (new FieldComparison<T> ("z", ComparisonOps::GT, _z_min)));
            _range_cond->addComparison (boost::shared_ptr< const FieldComparison<T> > (new FieldComparison<T> ("z", ComparisonOps::LT, _z_max)));

            _range_cond1.reset();	
            _range_cond1 = (boost::shared_ptr<ConditionAnd<T> >) new ConditionAnd<T>();
            _range_cond1->addComparison (boost::shared_ptr< const FieldComparison<T> > (new FieldComparison<T> ("z", ComparisonOps::GT, 0.55)));

        }

        void pointCloudCallback(const sensor_msgs::PointCloud2Ptr& pcmsg_in)
        {

            //STEP1: convert from ros msg to pcl (already removes RGB component because pc_in is pcl::PointXYZ)
            pcl::fromROSMsg(*pcmsg_in, *pc_in);
            //ROS_INFO_STREAM("Received a point cloud with stamp " << pcmsg_in->header.stamp.toSec() << " time now is " << ros::Time::now().toSec()); //some info


            //STEP2: transform to _fixed_frame_id
            tf::StampedTransform stf;
            try
            {
                _p_transform_listener->lookupTransform(_fixed_frame_id, pcmsg_in->header.frame_id , pcmsg_in->header.stamp, stf);
            }
            catch (tf::TransformException ex)
            {
                ROS_WARN("%s:\nCould not get transform: %s",_name.c_str(), ex.what());
                return;
            }

            Eigen::Affine3d eigen_trf;
            tf::transformTFToEigen (stf, eigen_trf);
            pcl::transformPointCloud<T>(*pc_in, *pc_transformed, eigen_trf);
            pc_transformed->header.frame_id = _fixed_frame_id;

            //STEP3: remove points outside the box
            ConditionalRemoval<T> condrem(_range_cond);
            condrem.setInputCloud(pc_transformed);
            condrem.setKeepOrganized(false);
            condrem.filter(*pc_filtered);
            //ROS_INFO_STREAM("After filtering point cloud has " << pc_filtered->points.size() << " points");

            //STEP4: voxelize
            if (_voxelize==true)
            {
                VoxelGrid<T> _vg;
                _vg.setInputCloud(pc_filtered);
                _vg.setLeafSize(_x_voxel, _y_voxel, _z_voxel);
                _vg.filter(*pc_downsampled);
                //ROS_INFO_STREAM("After dowsampling point cloud has " << pc_downsampled->points.size() << " points");
            }
            else
            {
                *pc_downsampled = *pc_filtered;
            }


            //STEP5: transform the point cloud back to the sensor frame_id
            pcl::transformPointCloud<T>(*pc_downsampled, *pc_downsampled, eigen_trf.inverse());
            pc_downsampled->header.frame_id = pcmsg_in->header.frame_id;

            //STEP6: Publish the point cloud
            toROSMsg(*pc_downsampled, msg_out);
            msg_out.header.frame_id = pcmsg_in->header.frame_id;
            msg_out.header.stamp = pcmsg_in->header.stamp;
            _p_pc_publisher->publish(msg_out);


            if (_flg_configure==true)
            {
                publishVisualizationMarkers();
            }
        }

        void makeMenuMarker( const tf::Vector3& position )
        {
            InteractiveMarker int_marker;
            int_marker.header.frame_id = _fixed_frame_id;
            tf::pointTFToMsg(position, int_marker.pose.position);
            int_marker.scale = 0.5;

            int_marker.name = "point_cloud_spatial_filter";
            int_marker.description = "Spatial Filter\n(Right Click)";

            InteractiveMarkerControl control;

            control.interaction_mode = InteractiveMarkerControl::MENU;
            control.name = "menu_only_control";

            Marker marker;

            marker.type = Marker::SPHERE;
            marker.scale.x = int_marker.scale * 0.65;
            marker.scale.y = int_marker.scale * 0.65;
            marker.scale.z = int_marker.scale * 0.65;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.color.a = 1.0;



            control.markers.push_back( marker );
            control.always_visible = true;
            int_marker.controls.push_back(control);

            server->insert(int_marker);
            server->setCallback(int_marker.name, boost::bind(&PerceptionPreprocessing::processFeedback, this, _1));
            menu_handler.apply( *server, int_marker.name );
        }


        /* _________________________________
           |                                 |
           |           ACCESSORS             |
           |_________________________________| */

};

#endif
