#ifndef LASER_BASED_RRT_STAR_H
#define LASER_BASED_RRT_STAR_H


#include "pugixml.hpp"
#include "limits.h"

//STL
#include <ctime>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <numeric>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include "cstdlib"
//egien
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include<Eigen/Core>
//opencv
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include "opencv2/core/core.hpp"
// ROS
#include "ros/ros.h"

//Drone module
#include "droneModuleROS.h"

//ROS message
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>

//Gazebo messages

#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"

//DroneMsgsROS
#include <droneMsgsROS/AliveSignal.h>
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/droneSpeeds.h>
#include <droneMsgsROS/droneTrajectoryRefCommand.h>
#include <droneMsgsROS/droneRefCommand.h>
#include <droneMsgsROS/dronePositionTrajectoryRefCommand.h>
#include <droneMsgsROS/dronePositionRefCommandStamped.h>
#include <droneMsgsROS/droneCommand.h>
#include <droneMsgsROS/droneYawRefCommand.h>

//Path_Smooth
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/trajectory_sampler_node.h>

#include <boost/thread/thread.hpp>
#include <set>
#include <rrtstar.h>
#include <dubins.h>


class LASER_BASED_RRT_STAR : public DroneModule
{
protected:
    struct {
        float pos_x_, pos_y_, pos_z_;
        float prev_pos_x_, prev_pos_y_, prev_pos_z_;
        float speed_x_, speed_y_, speed_z_;
        float prev_speed_x_, prev_speed_y_, prev_speed_z_;
        float roll_, pitch_, yaw_;
        float roll_norm_, pitch_norm_, yaw_norm_;
    }uav_state_;

    struct {
        float pos_x_, pos_y_, pos_z_;
    }target_state_;

    boost::mutex uav_mutex_;
    boost::mutex target_mutex_;
    ros::ServiceClient estimator_client_;
    ros::Publisher hector_mapping_reset_pub_;

    cv::Point3f uav_initial_reset_position_;

 ros::Subscriber drone_laser_scan_subs_;

 ros::Subscriber drone_pos_subs_;
 ros::Subscriber uav_pose_velocity_subs_;
 ros::Publisher traj_pub;
 ros::Publisher traj_control_pub;
 ros::Publisher obs_marker_pub;
 ros::Publisher position_control_pub;
 ros::Publisher planned_trajectory_out_pub_;
 ros::Publisher drone_state;

 ros::ServiceClient gazebo_set_model_state_srv_;
 cv::Mat dist_signed_real;

 droneMsgsROS::droneCommand drone_command;

 struct {
     float max_virtual_range_;
     float max_real_range_, min_real_range_;
     float min_range_reset_value_;
     int num_ranges_;
     int sampling_factor_;
     float angle_range_;
     float angle_sampling_factor_;
     float laser_state_normalization_factor_;

     std::vector<float> laser_ranges_;
     std::vector<float> min_laser_ranges_norm_;
 }laser_info_;

  struct {
     cv::Mat laser_scans_image_, obstacles_boundary_image_;
     cv::Size laser_scans_image_size_;
     float laser_scans_image_res_;
     float angle_ini_;
     float angle_ini_rad_;
     float angle_increment_;
     cv::Point p_origin_;
     std::vector<float> angles_ranges_;
     std::vector<float> cos_angles_ranges_;
     std::vector<float> sin_angles_ranges_;
 }laser_image_info_;

  struct {
      float pos_x_, pos_y_, pos_z_;
  }moving_obstacle_squared_;

 void LaserScanCallback(const sensor_msgs::LaserScan &msg);

 void dronePosCallback(const droneMsgsROS::dronePose &msg);

 void droneSpeedCallback(const droneMsgsROS::droneSpeeds &msg);

 bool ReadConfigs(std::string &configFile);

 float Get_Dist_fromSDF(float x_real, float y_real);

 std::vector<int> ComputeMinLaserRangesToObstacles();
 void SdffromLaser();

 void If_in_boundary();

 void Reset();

 void GenerateRandomPositions(cv::Point2f &uav_pose, cv::Point2f &target_pose);

 void PoseVelocityCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

 void SetUavState(const float x, const float y, const float z,const float dx, const float dy, const float dz);

 void SetUavPreviousState(const float x, const float y, const float z,
                                                   const float dx, const float dy, const float dz);
 void SetTargetState(const float x, const float y, const float z);

 void test(); //just used for testing sdf map



 private:
 std::vector<cv::Point3f> goals_sequence_;
 int num_goals_in_sequence_;
 bool min_distance_from_laser_flag_;
 bool imshow_obstacles_boundary_;

 visualization_msgs::Marker points;
 MTRand drand;
 float init_map_x;
 float init_map_y;
 float Xstartx;
 float Xstarty;


 cv::Point3f target_position;
 cv::Point3f uav_position, uav_linear_vel;
 int point_near;

  void DrawArrowedLine(cv::Mat& img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color,
                int thickness, int line_type, int shift, double tipLength);
  public:
    void open(ros::NodeHandle & nIn);
    void close();
public:
    bool init(std::string configFile);
    //void test_kdt(KDTree& tree,point_t& pt);//used for kdtree test
    //bool readConfigs(std::string configFile);
    //void readParameters();
public:
    bool run();
public:

    nav_msgs::Path rrt_star_path;
    nav_msgs::Path rrt_star_path_smooth;

    std::ofstream f_data_recorder;//record data

    DubinsPath path;
    geometry_msgs::Pose drone_pos;
    droneMsgsROS::droneSpeeds drone_speed;
    visualization_msgs::Marker points_laser_rt_rrt;
    vector<Node *> short_cut_path;
    ros::Time begin;

    float  min_range_t;
    int point_path_cur;
    bool path_plan_start;
    bool path_replan;
    bool position_pub;
    int goal_reached_;
    int num_episode_;
    //publish
    ros::Publisher rrt_star_publisher;
    ros::Publisher rrt_path_pub;
    ros::Publisher rrt_smooth_path_pub;
    ros::Publisher rrt_smooth_marker_pub;
    ros::Publisher yaw_pub;
    //subscribe
    ros::Subscriber rviz_point_sub;
    ros::Subscriber drone_control;
    ros::Subscriber drone_speed_subs_;

    droneMsgsROS::droneYawRefCommand drone_yaw_command;

    void initializeMarkers(visualization_msgs::Marker &sourcePoint,
                           visualization_msgs::Marker &goalPoint,
                           visualization_msgs::Marker &randomPoint,
                           visualization_msgs::Marker &rrtTreeMarker1,
                           visualization_msgs::Marker &rrtTreeMarker2,
                           visualization_msgs::Marker &finalPath,
                           visualization_msgs::Marker &finalSmoothPath
                            );

    void rrt_star_perform(geometry_msgs::Pose startpoint,geometry_msgs::Pose endpoint);

    float RRTNorm(std::vector<float> x1,std::vector<float> x2);

    float RRTsign(float n);

    void Path_Short(RRTSTAR &myRRT, float goalX, float goalY, float startX, float startY);
    float GetRandomReal(int low, int up);


    void Publish_P();
    void setFinalPathData(std::vector<Node *> &myRRT, visualization_msgs::Marker &finalpath, float goalX, float goalY);
    void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, Node *node1,Node *node2);
    void Path_Smooth(std::vector<Node *> &myRRT, visualization_msgs::Marker &finalpath, float goalX, float goalY,float startX, float startY);

    //Steer function prototype
    std::vector<float> RRTSteer(  std::vector<float>, std::vector<float>, float );
    void rviz_pathCallBack(const geometry_msgs::PointStamped &msg);
    void droneContrCallback(const droneMsgsROS::dronePose &msg);


    bool checkCollision(Vector2f &start, Vector2f &end, double eta);
    bool checkSample(Vector2f &n);

    void drone_position_control(nav_msgs::Path &myTrajectory, geometry_msgs::Point point_current_drone, geometry_msgs::Pose end, bool replan);
    void drone_tra_control(nav_msgs::Path &myTrajectory, geometry_msgs::Point point_current_drone, geometry_msgs::Pose end, bool replan);

    LASER_BASED_RRT_STAR();

    ~LASER_BASED_RRT_STAR();
};

#endif
