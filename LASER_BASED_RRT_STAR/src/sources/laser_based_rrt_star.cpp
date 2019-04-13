#include <laser_based_rrt_star.h>

static std::string const UAV_NAME = "hummingbird1";
static std::string const TARGET_NAME = "goal";

static double const kUav_Altitude_ = 1.2;

const float RRTSTAR_NEIGHBOR_FACTOR = 1;

const int maxNbFailures = 3;

LASER_BASED_RRT_STAR::LASER_BASED_RRT_STAR()
{

    //-----------------------------------------------------------//
    laser_info_.max_virtual_range_ = 2.0; //Maximum range for defining the laser components in the STATE.
    laser_info_.max_real_range_ = 30.0;
    laser_info_.min_real_range_ = 0.10;
    laser_info_.num_ranges_ = 720; //1079;
    laser_info_.angle_range_ = 270;
    laser_info_.sampling_factor_ = int(720/10.0); //72; //Sampling every 27 degrees
    //laser_info_.sampling_factor_ = 48; //Sampling every 18 degrees
    laser_info_.angle_sampling_factor_ = laser_info_.angle_range_ / (laser_info_.num_ranges_/laser_info_.sampling_factor_);


    laser_info_.min_range_reset_value_ = 0.3;
    laser_info_.laser_state_normalization_factor_ = laser_info_.max_virtual_range_ - laser_info_.min_range_reset_value_;



    laser_image_info_.angles_ranges_.clear();
    laser_image_info_.laser_scans_image_size_ = cv::Size(200, 200);
    laser_image_info_.angle_ini_ = -45.0;
    laser_image_info_.angle_ini_rad_ = laser_image_info_.angle_ini_*M_PI/180;
    laser_image_info_.angle_increment_ = laser_info_.angle_range_ / laser_info_.num_ranges_;
    laser_image_info_.p_origin_ = cv::Point(laser_image_info_.laser_scans_image_size_.width/2.0,
                                            laser_image_info_.laser_scans_image_size_.height/2.0);
    for(int i=0;i<laser_info_.num_ranges_;i++)
    {
        float angle_i = (laser_image_info_.angle_ini_ + i*laser_image_info_.angle_increment_) * M_PI/180.0;
        float cos_angle_i = std::cos(angle_i);
        float sin_angle_i = std::sin(angle_i);
        laser_image_info_.angles_ranges_.push_back(angle_i);
        laser_image_info_.cos_angles_ranges_.push_back(cos_angle_i);
        laser_image_info_.sin_angles_ranges_.push_back(sin_angle_i);
    }

    laser_image_info_.laser_scans_image_res_ = laser_info_.max_virtual_range_ / (laser_image_info_.laser_scans_image_size_.width/2.0);
    laser_image_info_.laser_scans_image_ = cv::Mat(laser_image_info_.laser_scans_image_size_.height,
                                                  laser_image_info_.laser_scans_image_size_.width,
                                                  CV_8U, cv::Scalar(255));
    laser_image_info_.obstacles_boundary_image_ = laser_image_info_.laser_scans_image_.clone();

//    dist_signed_real = cv::Mat(laser_image_info_.laser_scans_image_size_.height,
//                               laser_image_info_.laser_scans_image_size_.width,
//                               CV_64F, cv::Scalar(0));
}

LASER_BASED_RRT_STAR::~LASER_BASED_RRT_STAR()
{
//    f_data_recorder.close();

}

void LASER_BASED_RRT_STAR::open(ros::NodeHandle &nIn)
{
    //Node

    std::cout<<"Entering Open function"<<std::endl;
    DroneModule::open(nIn);

    //read the parameters from the launch file
   // readParameters();
    //Init
    //if(!init(stackPath+"configs/drone"+cvg_int_to_string(idDrone)+"/"+configFile))
    //{
     //   cout<<"Error init"<<endl;
      //  return;
    //}

    //// TOPICS
    //Subscribers
    drone_laser_scan_subs_ = n.subscribe("/drone7/scan",1, &LASER_BASED_RRT_STAR::LaserScanCallback,this);
    drone_pos_subs_ = n.subscribe("/drone7/EstimatedPose_droneGMR_wrt_GFF",1, &LASER_BASED_RRT_STAR::dronePosCallback,this);
    drone_speed_subs_ = n.subscribe("/drone7/EstimatedSpeed_droneGMR_wrt_GFF",1, &LASER_BASED_RRT_STAR::droneSpeedCallback,this);
    rviz_point_sub = n.subscribe("/drone7/clicked_point", 100 ,&LASER_BASED_RRT_STAR::rviz_pathCallBack,this);
//    drone_control = n.subscribe("/drone7/EstimatedPose_droneGMR_wrt_GFF",1, &LASER_BASED_RRT_STAR::droneContrCallback, this);
    uav_pose_velocity_subs_ = n.subscribe("/gazebo/model_states", 10, &LASER_BASED_RRT_STAR::PoseVelocityCallback, this);

    //Publishers
    rrt_star_publisher = n.advertise<visualization_msgs::Marker>("path",1,true);
    rrt_smooth_marker_pub = n.advertise<visualization_msgs::Marker>("path_smooth",1,true);
    position_control_pub = n.advertise<droneMsgsROS::dronePositionRefCommandStamped>("/drone7/dronePositionRefs",1);
    hector_mapping_reset_pub_ = n.advertise<std_msgs::String>("syscommand", 1000);
    drone_state = n.advertise<droneMsgsROS::droneCommand>("/drone7/command/high_level",1,true);
    yaw_pub = n.advertise<droneMsgsROS::droneYawRefCommand>("/drone7/droneControllerYawRefCommand",1,true);
    traj_control_pub = n.advertise<droneMsgsROS::dronePositionTrajectoryRefCommand>("/drone7/droneTrajectoryAbsRefCommand", 1);
    //hector_mapping_reset_pub_ = n.advertise<std_msgs::String>(param_string, 1000);
    //Services

    gazebo_set_model_state_srv_ = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    estimator_client_ = n.serviceClient<std_srvs::Empty>("/drone7/droneRobotLocalizationROSModuleNode/reset");
    //Flag of module opened
    droneModuleOpened=true;
    //autostart module!
    moduleStarted=false;


    imshow_obstacles_boundary_ = true;

//----------------------------------------------------------------initial param----------------------------------------------------------------
//    Reset();
//----------------------------------------------------------------initial param----------------------------------------------------------------
    std::string param_string;
    ros::param::get("~configs_path", param_string);
   if(param_string.length() == 0)
   {
       ROS_ERROR("FATAL: Topic not found");
       return;
   }
   std::string configs_file_name = param_string;
   std::cout<<"++++++++++ CONFIGS PATH ++++++++++"<<std::endl<<configs_file_name<<std::endl;
   ReadConfigs(configs_file_name);
   path_plan_start = false;
   path_replan = false;
   point_path_cur = 0;
   position_pub = true;
   goal_reached_ = 0;
   num_episode_ = 1;
   point_near = 2;
   Reset();
    //End
//------------------------------------------------------------------data record---------------------------------------------------------------------//
   f_data_recorder.open("/home/liang/data_collision_avoidance_env2/datanenv2.txt");
   f_data_recorder<<"time"<<"\t"<<"num_episode"<<"\t"<<"goal_reached"<<"\t"<<"uav_posx"<<"\t"<<"uav_posy"<<"\t"<<"target_posx"<<"\t"<<
                        "target_posy"<<"\t"<<"moving_obstacle_posx"<<"\t"<<"moving_obstacle_posy"<<"\t"<<
                        "speed_x"<<"\t"<<"speed_y"<<"\t"<<"min_range"<<std::endl;
//--------------------------------------------------------------------------------------------------------------------------------------------------//
    return;
}

void LASER_BASED_RRT_STAR::close()
{
    DroneModule::close();

    //Do stuff

    return;
}


bool LASER_BASED_RRT_STAR::run()
{
    if(!DroneModule::run())
        return false;

    if(droneModuleOpened==false)
        return false;

    return true;
}

void LASER_BASED_RRT_STAR::LaserScanCallback(const sensor_msgs::LaserScan &msg)
{
    laser_info_.laser_ranges_.clear();
    laser_info_.laser_ranges_.insert(laser_info_.laser_ranges_.end(), msg.ranges.begin(), msg.ranges.end());

     min_range_t = *std::min_element(laser_info_.laser_ranges_.begin(), laser_info_.laser_ranges_.end());

        if ( std::any_of(laser_info_.laser_ranges_.begin(), laser_info_.laser_ranges_.end(), [this](float i){return i<laser_info_.min_range_reset_value_;}) )
        {
    //        min_distance_from_laser_flag_ = true;

            std::cout<<"MIN DISTANCE reached!"<<std::endl;
            path_plan_start = true;
            path_replan = false;
            point_path_cur = 0;
            position_pub = true;
            num_episode_ = num_episode_ + 1;
            goal_reached_ = 0;
            point_near = 2;

            rrt_star_path_smooth.poses.clear();
            rrt_star_path.poses.clear();

            Reset();

//            droneMsgsROS::dronePositionRefCommandStamped position_cmd;

//            position_cmd.position_command.x = drone_pos.position.x;
//            position_cmd.position_command.y = drone_pos.position.y;
//            position_cmd.position_command.z = kUav_Altitude_;

//            position_control_pub.publish(position_cmd);



        }

    SdffromLaser();

}

void LASER_BASED_RRT_STAR::dronePosCallback(const droneMsgsROS::dronePose &msg)
{
    drone_pos.position.x = msg.x;
    drone_pos.position.y = msg.y;
    drone_pos.position.z = msg.z;
    drone_pos.orientation.w = 1;
    drone_pos.orientation.x = 0;
    drone_pos.orientation.y = 0;
    drone_pos.orientation.z = 0;
}

void LASER_BASED_RRT_STAR::droneSpeedCallback(const droneMsgsROS::droneSpeeds &msg)
{
    drone_speed.dx = msg.dx;
    drone_speed.dy = msg.dy;
    drone_speed.dyaw = msg.dyaw;

}

//void LASER_BASED_RRT_STAR::droneContrCallback(const droneMsgsROS::dronePose &msg)
//{

//    geometry_msgs::Point current_drone_point;

//    current_drone_point.x = msg.x;
//    current_drone_point.y = msg.y;

//    geometry_msgs::Pose p_init;
//    geometry_msgs::Pose p_end;
//    p_end.position.x = 5;
//    p_end.position.y =1;


////    float max_x,max_y,min_x,min_y;


//    if(path_plan_start == true || path_replan == true)
//     {
//        p_init.position.x = msg.x;
//        p_init.position.y = msg.y;
//       clock_t startTime,endTime;
//       startTime = clock();
//       rrt_star_perform(p_init, p_end);
//       endTime = clock();
//       std::cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
//       }

//    if(rrt_star_path_smooth.poses.size()>0)
//      {
//        drone_position_control(rrt_star_path_smooth,current_drone_point,p_end,path_replan);
//        path_plan_start = false;
//        path_replan = false;

//    }

//  if(rrt_star_path_smooth.poses.size() > 0)
//   {
//    Vector2f last_path_point;

//    Vector2f next_path_point;

//    if((point_path_cur+3) < rrt_star_path_smooth.poses.size())
//     {
//        last_path_point.x() = rrt_star_path_smooth.poses[point_path_cur].pose.position.x;
//        last_path_point.y() = rrt_star_path_smooth.poses[point_path_cur].pose.position.y;

//        next_path_point.x() = rrt_star_path_smooth.poses[point_path_cur+3].pose.position.x;
//        next_path_point.y() = rrt_star_path_smooth.poses[point_path_cur+3].pose.position.y;

//        Vector2f v = last_path_point - next_path_point;
//        double dist = sqrt(pow(v.x(), 2) + pow(v.y(), 2));


//        if(!checkCollision(last_path_point,next_path_point,dist))
//          {

//            path_replan = true;
//            rrt_star_path_smooth.poses.clear();
//            rrt_star_path.poses.clear();

//        }

//    }
////    for(int i = 0; i < (int)rrt_star_path.poses.size()-1; i++) {

////        if(rrt_star_path.poses[i].pose.position.x > rrt_star_path.poses[i+1].pose.position.x)
////          {
////            max_x = rrt_star_path.poses[i].pose.position.x;
////            min_x = rrt_star_path.poses[i+1].pose.position.x;
////        }
////        else
////          {
////            min_x = rrt_star_path.poses[i].pose.position.x;
////            max_x = rrt_star_path.poses[i+1].pose.position.x;
////        }


////        if(rrt_star_path.poses[i].pose.position.y > rrt_star_path.poses[i+1].pose.position.y)
////          {
////            max_y = rrt_star_path.poses[i].pose.position.y;
////            min_y = rrt_star_path.poses[i+1].pose.position.y;
////        }
////        else
////          {
////            min_y = rrt_star_path.poses[i].pose.position.y;
////            max_y = rrt_star_path.poses[i+1].pose.position.y;
////        }


////      if(msg.x < max_x && msg.x > min_x && msg.y > min_y && msg.y < max_y)
////       {
////        last_path_point.x() = rrt_star_path.poses[i].pose.position.x;
////        last_path_point.y() = rrt_star_path.poses[i].pose.position.y;

////        next_path_point.x() = rrt_star_path.poses[i+1].pose.position.x;
////        next_path_point.y() = rrt_star_path.poses[i+1].pose.position.y;

////        Vector2f v = last_path_point - next_path_point;
////        double dist = sqrt(pow(v.x(), 2) + pow(v.y(), 2));

//////        if(checkCollision(last_path_point,next_path_point,dist))
//////           {
//////            std::cout<<"no collision!!"<<std::endl;
//////        }
//////        else
//////            {
//////            std::cout<<"collision!!"<<std::endl;
//////        }

////        if(!checkCollision(last_path_point,next_path_point,dist))
////          {

////            path_replan = true;
////            rrt_star_path_smooth.poses.clear();
////            rrt_star_path.poses.clear();

////        }

////      }





////    }
//  }

//}

bool LASER_BASED_RRT_STAR::ReadConfigs(std::string &configFile)
{
    pugi::xml_document doc;
    std::ifstream nameFile(configFile.c_str());
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
    {
        std::cout << "ERROR: Could not load the file: " << result.description() << std::endl;
        return 0;
    }

    pugi::xml_node Configuration = doc.child("Laser_Based_Navigation_Config");
    std::string readingValue;

    readingValue = Configuration.child("laser_info").child_value("max_virtual_range");
    laser_info_.max_virtual_range_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.max_virtual_range_: "<<laser_info_.max_virtual_range_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("max_real_range");
    laser_info_.max_real_range_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.max_real_range_: "<<laser_info_.max_real_range_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("min_real_range");
    laser_info_.min_real_range_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.min_real_range_: "<<laser_info_.min_real_range_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("angle_range");
    laser_info_.angle_range_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.angle_range_: "<<laser_info_.angle_range_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("num_ranges");
    laser_info_.num_ranges_ = atoi(readingValue.c_str());
    std::cout<<"laser_info_.num_ranges_: "<<laser_info_.num_ranges_<<std::endl;


    readingValue = Configuration.child("laser_info").child_value("min_range_reset_value");
    laser_info_.min_range_reset_value_ = atof(readingValue.c_str());
    std::cout<<"laser_info_.min_range_reset_value_: "<<laser_info_.min_range_reset_value_<<std::endl;




    laser_info_.laser_state_normalization_factor_ = laser_info_.max_virtual_range_ - laser_info_.min_range_reset_value_;




    laser_info_.laser_state_normalization_factor_ = laser_info_.max_virtual_range_ - laser_info_.min_range_reset_value_;



    laser_image_info_.angles_ranges_.clear();
    laser_image_info_.laser_scans_image_size_ = cv::Size(200, 200);
    laser_image_info_.angle_ini_ = -45.0;
    laser_image_info_.angle_ini_rad_ = laser_image_info_.angle_ini_*M_PI/180;
    laser_image_info_.angle_increment_ = laser_info_.angle_range_ / laser_info_.num_ranges_;
    laser_image_info_.p_origin_ = cv::Point(laser_image_info_.laser_scans_image_size_.width/2.0,
                                            laser_image_info_.laser_scans_image_size_.height/2.0);
    for(int i=0;i<laser_info_.num_ranges_;i++)
    {
        float angle_i = (laser_image_info_.angle_ini_ + i*laser_image_info_.angle_increment_) * M_PI/180.0;
        float cos_angle_i = std::cos(angle_i);
        float sin_angle_i = std::sin(angle_i);
        laser_image_info_.angles_ranges_.push_back(angle_i);
        laser_image_info_.cos_angles_ranges_.push_back(cos_angle_i);
        laser_image_info_.sin_angles_ranges_.push_back(sin_angle_i);
    }

    laser_image_info_.laser_scans_image_res_ = laser_info_.max_virtual_range_ / (laser_image_info_.laser_scans_image_size_.width/2.0);
    laser_image_info_.laser_scans_image_ = cv::Mat(laser_image_info_.laser_scans_image_size_.height,
                                                  laser_image_info_.laser_scans_image_size_.width,
                                                  CV_8U, cv::Scalar(255));
    laser_image_info_.obstacles_boundary_image_ = laser_image_info_.laser_scans_image_.clone();

    return 1;

//    if(STATE_BASED_ON_GROUP_OF_LASER_RANGES)
//        environment_info_.state_dim_low_dim_ = 4 + (laser_info_.num_ranges_ / laser_info_.sampling_factor_);
//    else
//        environment_info_.state_dim_low_dim_ = 4 + (laser_info_.num_ranges_ / laser_info_.sampling_factor_) + 1;


}

void LASER_BASED_RRT_STAR::DrawArrowedLine(cv::Mat& img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color,
           int thickness, int line_type, int shift, double tipLength)
{

    const double tipSize = 50 * tipLength; // Factor to normalize the size of the tip depending on the length of the arrow

    cv::line(img, pt1, pt2, color, thickness, line_type, shift);

    const double angle = atan2( (double) pt1.y - pt2.y, (double) pt1.x - pt2.x );

    cv::Point p(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)),
        cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
    cv::line(img, p, pt2, color, thickness, line_type, shift);

    p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
    p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
    cv::line(img, p, pt2, color, thickness, line_type, shift);
}


//-----------------------------------------------------------------sdf_from_laser--------------------------------------------------------------------------------
void LASER_BASED_RRT_STAR::SdffromLaser()
{
    if(1)
    {
        std::vector<float> all_saturated_ranges;
        all_saturated_ranges.reserve(laser_info_.laser_ranges_.size());
        for(size_t i=0;i<laser_info_.laser_ranges_.size();i++)
        {
            if(std::isfinite(laser_info_.laser_ranges_[i]))
            {
                if(laser_info_.laser_ranges_[i] > laser_info_.max_virtual_range_)
                    all_saturated_ranges.push_back(laser_info_.max_virtual_range_);
                else
                    all_saturated_ranges.push_back(laser_info_.laser_ranges_[i]);
            }
            else
                all_saturated_ranges.push_back(laser_info_.max_virtual_range_);
        }


        cv::Mat I_all_saturated_ranges = laser_image_info_.laser_scans_image_.clone();
        cv::cvtColor(I_all_saturated_ranges, I_all_saturated_ranges, CV_GRAY2BGR);
        for(size_t i=0;i<all_saturated_ranges.size();i++)
        {
            float delta_pos_proyec_module = all_saturated_ranges[i]/laser_image_info_.laser_scans_image_res_;
            cv::Point2f delta_pos_proyec;
            delta_pos_proyec.x = delta_pos_proyec_module * laser_image_info_.cos_angles_ranges_[i];
            delta_pos_proyec.y = -delta_pos_proyec_module * laser_image_info_.sin_angles_ranges_[i];

            int x_im = laser_image_info_.laser_scans_image_.cols/2 + delta_pos_proyec.x;
            int y_im = laser_image_info_.laser_scans_image_.rows/2 + delta_pos_proyec.y;

            cv::Point p_fin = cv::Point(x_im, y_im);
            cv::line(I_all_saturated_ranges, laser_image_info_.p_origin_, p_fin, cv::Scalar(0), 1);
            if((i%laser_info_.sampling_factor_ == 0) || (i == all_saturated_ranges.size()-1))
                cv::line(I_all_saturated_ranges, laser_image_info_.p_origin_, p_fin, cv::Scalar(255, 0, 0), 2);
        }

        cv::imshow("All saturated ranges", I_all_saturated_ranges);
        cv::waitKey(1);

    }

    cv::Mat I_obstacles_boundary = laser_image_info_.obstacles_boundary_image_.clone();
    int circle_radius = 10;
    for(size_t i=0;i<laser_info_.laser_ranges_.size();i++)
    {
        float delta_pos_proyec_module = laser_info_.laser_ranges_[i]/laser_image_info_.laser_scans_image_res_;
        cv::Point2f delta_pos_proyec;
        delta_pos_proyec.x = delta_pos_proyec_module * laser_image_info_.cos_angles_ranges_[i];
        delta_pos_proyec.y = -delta_pos_proyec_module * laser_image_info_.sin_angles_ranges_[i];

        int x_im = laser_image_info_.laser_scans_image_.cols/2 + delta_pos_proyec.x;
        int y_im = laser_image_info_.laser_scans_image_.rows/2 + delta_pos_proyec.y;

        cv::Point p_range_i = cv::Point(x_im, y_im);
        cv::circle(I_obstacles_boundary, p_range_i, circle_radius, cv::Scalar(0), -1);
    }


    //Find contours in image in order to compute the number of obstacles
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat I_obstacles_vector_rgb = I_obstacles_boundary.clone();
    cv::bitwise_not(I_obstacles_vector_rgb, I_obstacles_vector_rgb);
    cv::Mat I_for_contours = I_obstacles_vector_rgb.clone();
    cv::findContours(I_for_contours, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    //Remove those obstacles that are very small (only consider obstacles > than a predefined area)
    std::vector<std::vector<cv::Point> >::iterator itc = contours.begin();
    itc = contours.begin();
    while(itc!=contours.end())
    {
        cv::Rect box = cv::boundingRect(cv::Mat(*itc));
        if(box.area() < 400)
            itc = contours.erase(itc);
        else
            ++itc;
    }

    cv::cvtColor(I_obstacles_vector_rgb, I_obstacles_vector_rgb, CV_GRAY2RGB);
//calculate distance map from I_obstacles_vecror_rgd
//    clock_t startTime,endTime;
//    startTime = clock();

    cv::Mat dist1, dist2,dview,dist_signed,dis_signed_imshow,dview_uchar;
    cv::Mat temporal_image,dist_signed_pic;

    int maskSize0 = cv::DIST_MASK_PRECISE;
    int distType0 = cv::DIST_L2;
    cv::cvtColor(I_obstacles_vector_rgb, temporal_image, CV_RGB2GRAY);

//    //distanceTransform
    cv::distanceTransform( temporal_image, dist1, distType0, maskSize0 );
    cv::distanceTransform( 255-temporal_image, dist2, distType0, maskSize0 );

    dist_signed_pic=dist2-dist1;
    double minv = 0.0, maxv = 0.0;
    double* minp = &minv;
    double* maxp = &maxv;
    cv::minMaxIdx(dist_signed_pic,minp,maxp);

    if(maxv-minv==0&&maxv<0)
    {
        dist_signed = dist_signed_pic*0;
        dist_signed_real = -FLT_MAX*(dist_signed+1);
    }
   if(maxv-minv==0&&maxv>0)
    {
        dist_signed =dist_signed_pic*0+1;
        dist_signed_real = FLT_MAX*(dist_signed);
    }
    if(maxv-minv>0)
    {
        dist_signed = (dist_signed_pic-minv)/(maxv-minv);
        dist_signed_real = dist_signed_pic*0.02;//0.02m is the resolution of the picture
    }

    //imshow the sdf map
    dis_signed_imshow = dist_signed;

    dis_signed_imshow.convertTo(dview_uchar,CV_8U,255);

    cv::applyColorMap(dview_uchar,dview,cv::COLORMAP_HOT);

    cv::imshow("temporal_image", temporal_image);
    cv::imshow("sdf_map", dview);
//    endTime = clock();
//    std::cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

}

float LASER_BASED_RRT_STAR::Get_Dist_fromSDF(float x_real, float y_real)
{
  float image_point_y,image_point_x;
  float obs_distance;
  //the resolution of map is 0.02
  //transfer from real position to image pixel
  image_point_y = 50*drone_pos.position.x - 50* x_real+100;
  image_point_x = 50*drone_pos.position.y - 50* y_real+100;
  //obs_distance from signed distance field
  float x_sdf = x_real - drone_pos.position.x;
  float y_sdf = y_real - drone_pos.position.y;
  if(x_sdf>= -2 && x_sdf <= 2&&y_sdf >= -2&&y_sdf<= 2)
 {
  obs_distance = dist_signed_real.at<float>(round(image_point_y),round(image_point_x));
 }
  else
    {
      obs_distance = 0.2;
  }


  return obs_distance;


}

void LASER_BASED_RRT_STAR::test()
{

    SdffromLaser();

    float dist = Get_Dist_fromSDF(-2.7881,3.08277);

    std::cout<<"dist to obs = "<<dist<<std::endl;//radius of obstacle cylind is 0.25m
}

void LASER_BASED_RRT_STAR::initializeMarkers(visualization_msgs::Marker &sourcePoint,
                                 visualization_msgs::Marker &goalPoint,
                                 visualization_msgs::Marker &randomPoint,
                                 visualization_msgs::Marker &rrtTreeMarker1,
                                 visualization_msgs::Marker &rrtTreeMarker2,
                                 visualization_msgs::Marker &finalPath
                                , visualization_msgs::Marker &finalSmoothPath)
{
    //init headers
    sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker1.header.frame_id = rrtTreeMarker2.header.frame_id  = finalPath.header.frame_id    = finalSmoothPath.header.frame_id  = "map";
    sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker1.header.stamp    = rrtTreeMarker2.header.stamp     = finalPath.header.stamp       = finalSmoothPath.header.stamp = ros::Time::now();
    sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker1.ns              = rrtTreeMarker2.ns               = finalPath.ns                 = finalSmoothPath.ns ="map";
    sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker1.action          = rrtTreeMarker2.action           = finalPath.action             = finalSmoothPath.action =visualization_msgs::Marker::ADD;
    sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker1.pose.orientation.w = rrtTreeMarker2.pose.orientation.w  = finalPath.pose.orientation.w = finalSmoothPath.pose.orientation.w = 1.0;
    //setting id for each marker
    sourcePoint.id    = 0;
    goalPoint.id      = 1;
    randomPoint.id    = 2;
    rrtTreeMarker1.id = 3;
    finalPath.id      = 4;
    rrtTreeMarker2.id = 5;
    finalSmoothPath.id = 6;

    //defining types
    rrtTreeMarker1.type                                   = visualization_msgs::Marker::LINE_LIST;
    rrtTreeMarker2.type                                   = visualization_msgs::Marker::LINE_LIST;
    finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
    finalSmoothPath.type                                  = visualization_msgs::Marker::LINE_STRIP;
    sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

    //setting scale
    rrtTreeMarker1.scale.x = 0.05;
    rrtTreeMarker2.scale.x = 0.05;
    finalPath.scale.x     = 0.05;
    finalSmoothPath.scale.x = 0.05;
    sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 0.25;
    sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 0.25;
    sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 0.1;

    //assigning colors
    sourcePoint.color.r   = 1.0f;
    goalPoint.color.g     = 1.0f;
    randomPoint.color.b   = 1.0f;

    rrtTreeMarker1.color.r = 0.8f;
    rrtTreeMarker1.color.g = 0.4f;

    rrtTreeMarker2.color.r = 0.8f;
    rrtTreeMarker2.color.g = 0.4f;

    finalPath.color.r = 0.0f;
    finalPath.color.g = 0.0f;
    finalPath.color.b = 0.0f;

    finalSmoothPath.color.r = 1.0;
    finalSmoothPath.color.g = 0;
    finalSmoothPath.color.b = 0;

    sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker1.color.a= rrtTreeMarker2.color.a = finalPath.color.a = finalSmoothPath.color.a = 1.0f;

}

void LASER_BASED_RRT_STAR::addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker,Node *node1,Node *node2)
{
    geometry_msgs::Point pointChild;
    geometry_msgs::Point pointParent;

                pointParent.x = node1->position.x();
                pointParent.y = node1->position.y();
                pointParent.z = 0;
               rrtTreeMarker.points.push_back(pointParent);
                pointChild.x = node2->position.x();
                pointChild.y = node2->position.y();
                pointChild.z = 0;
               rrtTreeMarker.points.push_back(pointChild);

}

//-----------------------------------------------------random based path short--------------------------------------------------------------------------------//

void LASER_BASED_RRT_STAR::Path_Short(RRTSTAR &myRRT,float goalX, float goalY, float startX, float startY)
{
    int nbFailures = 0;


    srand (time(NULL));

    while(nbFailures < maxNbFailures)
    {
        bool failure = true;
        int RRT_path_size = myRRT.path.size();
        short_cut_path.clear();

        if(RRT_path_size ==2)
         {
            break;
        }

        float x_low = GetRandomReal(1,RRT_path_size-2);

        int X1 = round(x_low);

//        float x_high = GetRandomReal(X1,RRT_path_size-1);

//        int X2 = round(x_high);

        Vector2f start0(startX,startY);
        Vector2f end2(goalX,goalY);
        Vector2f end0(myRRT.path[RRT_path_size-1-X1]->position.x(),myRRT.path[RRT_path_size-1-X1]->position.y());
//        Vector2f end1(myRRT.path[RRT_path_size-1-X2]->position.x(),myRRT.path[RRT_path_size-1-X2]->position.y());


        float l01 = pow(	(pow(abs((end0.x()-start0.x())),2)+pow(abs((end0.y()-start0.y())),2))	,0.5);
        float l12 = pow(	(pow(abs((end2.x()-end0.x())),2)+pow(abs((end2.y()-end0.y())),2))	,0.5);
//        float l23 = pow(	(pow(abs((end2.x()-end1.x())),2)+pow(abs((end2.y()-end1.y())),2))	,0.5);

//        if(checkCollision(end1,end2,l23))
//        {
//          Node* node0;
//          node0 = new Node;
//          node0->position.x() = end1.x();
//          node0->position.y() = end1.y();
//          short_cut_path.push_back(node0);
//          failure = false;
//          std::cout<<"node0_x = "<<node0->position.x()<<std::endl;

//          std::cout<<"node0_y = "<<node0->position.y()<<std::endl;
//        }
//        else
//        {
//          for(int i = 0;i< RRT_path_size-X2-1;i++)
//          {
//            short_cut_path.push_back(myRRT.path[i]);
//            std::cout<<" x_0 ="<<myRRT.path[i]->position.x()<<std::endl;
//            std::cout<<" y_0 ="<<myRRT.path[i]->position.y()<<std::endl;

//          }

//        }

        if(checkCollision(end0,end2,l12))
        {

                Node* node1;
                node1 = new Node;
                node1->position.x() = end0.x();
                node1->position.y() = end0.y();
                short_cut_path.push_back(node1);
                failure = false;
        }
        else
        {
            for(int i = 0;i< RRT_path_size-X1-1;i++)
            {
              short_cut_path.push_back(myRRT.path[i]);

            }

        }



        if(checkCollision(start0,end0,l01))
        {


            Node* node1;
            node1 = new Node;
            node1->position.x() = start0.x();
            node1->position.y() = start0.y();
            short_cut_path.push_back(node1);
            failure = false;

        }
        else
        {
            for(int i = RRT_path_size-X1;i< RRT_path_size;i++)
            {
              short_cut_path.push_back(myRRT.path[i]);

            }

        }

        myRRT.path.clear();
        for(int j = 0; j<short_cut_path.size(); j++)
        {

            myRRT.path.push_back(short_cut_path[j]);

        }

      if(RRT_path_size < 4)
       {

           nbFailures = maxNbFailures;

       }
      else
        {
          if(failure)
          {
             nbFailures = nbFailures + 1;

          }

        }

    }

}
//----------------------------------------------------------------------------------------------------------------------------------------------------//

float LASER_BASED_RRT_STAR::GetRandomReal(int low, int up)
{

    float fResult;
    if (low > up)
    {
      int temp = low;
      low = up;
      up = temp;
    }
    fResult = low + rand()%1000 /1000.0*(up - low);
    return fResult;

}

void LASER_BASED_RRT_STAR::setFinalPathData( std::vector<Node *> &myRRT, visualization_msgs::Marker &finalpath,float goalX,float goalY)
{

    geometry_msgs::Point point;
    geometry_msgs::PoseStamped pose_rrt;
        point.x = goalX;
        point.y = goalY;
        point.z =0;
        rrt_star_path.header.frame_id = "map";

        pose_rrt.pose.position= point;
        pose_rrt.pose.orientation.x = 0;
        pose_rrt.pose.orientation.y = 0;
        pose_rrt.pose.orientation.z = 0;
        pose_rrt.pose.orientation.w = 1;
        rrt_star_path.poses.push_back(pose_rrt);
        finalpath.points.push_back(point);
    for(int i=0; i<myRRT.size();i++)
    {

      {
        point.x = myRRT[i]->position.x();
        point.y = myRRT[i]->position.y();
        point.z = 0;
        pose_rrt.pose.position= point;
        pose_rrt.pose.orientation.x = 0;
        pose_rrt.pose.orientation.y = 0;
        pose_rrt.pose.orientation.z = 0;
        pose_rrt.pose.orientation.w = 1;
        finalpath.points.push_back(point);
        rrt_star_path.poses.push_back(pose_rrt);
      }

    }
}
//------------------------------------------------------------------------------------minimun snap trajectory optim------------------------------------------------------------------------------
void LASER_BASED_RRT_STAR::Path_Smooth(std::vector<Node *> &myRRT, visualization_msgs::Marker &finalpath, float goalX, float goalY, float startX, float startY)
{

    mav_trajectory_generation::Vertex::Vector vertices;

    visualization_msgs::Marker startMarker, goalMarker;

    const int dimension = 4;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
    int path_num = myRRT.size();
    for(int n = 0; n < myRRT.size();n++ )
   {
        if (n == 0) {
            start.makeStartOrEnd(Eigen::Vector4d(startX, startY, 0, 0), derivative_to_optimize);
            vertices.push_back(start);
            startMarker.id = 0;
            startMarker.type = visualization_msgs::Marker::SPHERE;
            startMarker.action = visualization_msgs::Marker::ADD;
            startMarker.pose.position.x = startX;
            startMarker.pose.position.y = startY;
            startMarker.pose.position.z = 0;
            startMarker.pose.orientation.x = 0.0;
            startMarker.pose.orientation.y = 0.0;
            startMarker.pose.orientation.z = 0.0;
            startMarker.pose.orientation.w = 1.0;
            startMarker.scale.x = 0.5;
            startMarker.scale.y = 0.5;
            startMarker.scale.z = 0.5;
            startMarker.color.a = 1.0; // Don't forget to set the alpha!
            startMarker.color.r = 0.0;
            startMarker.color.g = 0.0;
            startMarker.color.b = 1.0;

        }
        else {
            middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(myRRT[path_num-1-n]->position.x(), myRRT[path_num-1-n]->position.y(), 0, 0));
            vertices.push_back(middle);
        }


    }


    end.makeStartOrEnd(Eigen::Vector4d(goalX, goalY, 0, 0), derivative_to_optimize);
    vertices.push_back(end);
    goalMarker.id = 0;
    goalMarker.type = visualization_msgs::Marker::SPHERE;
    goalMarker.action = visualization_msgs::Marker::ADD;
    goalMarker.pose.position.x = 4;
    goalMarker.pose.position.y = 0;
    goalMarker.pose.position.z = 0;
    goalMarker.pose.orientation.x = 0.0;
    goalMarker.pose.orientation.y = 0.0;
    goalMarker.pose.orientation.z = 0.0;
    goalMarker.pose.orientation.w = 1.0;
    goalMarker.scale.x = 0.5;
    goalMarker.scale.y = 0.5;
    goalMarker.scale.z = 0.5;
    goalMarker.color.a = 1.0; // Don't forget to set the alpha!
    goalMarker.color.r = 0.0;
    goalMarker.color.g = 1.0;
    goalMarker.color.b = 0.0;


    std::vector<double> segment_times;
    const double v_max = 2.0;
    const double a_max = 2.0;
    const double magic_fabian_constant = 6.5; // A tuning parameter.
    //segment_times =estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);

    segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, v_max, a_max);
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 500.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;

    const int N = 10;
    //mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters, false);
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

    ROS_INFO("Performing optimization...");
    opt.optimize();

    mav_trajectory_generation::Segment::Vector segments;
    opt.getPolynomialOptimizationRef().getSegments(&segments);
    ROS_INFO("Done.");
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);

    std::string frame_id = "map";
    std_msgs::Header header;
    header.frame_id = frame_id;

    mav_msgs::EigenTrajectoryPoint::Vector states;

    double sampling_interval = 0.1;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

    geometry_msgs::Point point;
    geometry_msgs::PoseStamped pose_rrt;
    rrt_star_path_smooth.header.frame_id = "map";
    for(int i=0; i<states.size()-1;i++)
    {

      {

        point.x = states[i].position_W[0];
        point.y = states[i].position_W[1];
        point.z = 0;
        pose_rrt.pose.position= point;
        pose_rrt.pose.orientation.x = 0;
        pose_rrt.pose.orientation.y = 0;
        pose_rrt.pose.orientation.z = 0;
        pose_rrt.pose.orientation.w = 1;
        finalpath.points.push_back(point);
        rrt_star_path_smooth.poses.push_back(pose_rrt);
      }

    }

}

//-----------------------------------------------------------------position based control system--------------------------------------------------------------------------------------------//

void LASER_BASED_RRT_STAR::drone_position_control(nav_msgs::Path &myTrajectory, geometry_msgs::Point point_current_drone ,geometry_msgs::Pose end,bool replan)
{
    int traj_num;
    traj_num = myTrajectory.poses.size();

    droneMsgsROS::dronePositionRefCommandStamped position_cmd;

   double dist_deone_to_end = sqrt(pow((end.position.x-point_current_drone.x), 2) + pow((end.position.y-point_current_drone.y), 2));
    if(replan == false || path_plan_start == true)
   {
    position_cmd.position_command.x = myTrajectory.poses[point_path_cur+5].pose.position.x;
    position_cmd.position_command.y = myTrajectory.poses[point_path_cur+5].pose.position.y;
    position_cmd.position_command.z = kUav_Altitude_;

//    double deltay = point_current_drone.y-myTrajectory.poses[point_path_cur+1].pose.position.y;
//    double deltax = point_current_drone.x-myTrajectory.poses[point_path_cur+1].pose.position.x;

//    drone_yaw_command.yaw = atan2(deltay,deltax)+3.1415926;

    if(position_pub == true && dist_deone_to_end > 0.1)
    {
    position_control_pub.publish(position_cmd);
//    yaw_pub.publish(drone_yaw_command);
    position_pub = false;
    }
    double dist_drone_to_point = sqrt(pow((myTrajectory.poses[point_path_cur+5].pose.position.x-point_current_drone.x), 2) + pow((myTrajectory.poses[point_path_cur+5].pose.position.y-point_current_drone.y), 2));
    if(dist_drone_to_point < 0.3)
    point_path_cur = point_path_cur+5;
    position_pub = true;
    }

    if(replan == true)
    {
      point_path_cur = 0;
      position_pub = true;
    }
    if(point_path_cur >= traj_num && dist_deone_to_end >0.1)
    {
       point_path_cur = 0;
       path_plan_start = true;
       rrt_star_path_smooth.poses.clear();
       rrt_star_path.poses.clear();
       position_pub = true;
    }


}
//------------------------------------------------------------------------------------mpc_trajectory_controller----------------------------------------------------------------------------------------------------//
void LASER_BASED_RRT_STAR::drone_tra_control(nav_msgs::Path &myTrajectory, geometry_msgs::Point point_current_drone, geometry_msgs::Pose end, bool replan)
{
    int traj_num;
    traj_num = myTrajectory.poses.size();

    droneMsgsROS::dronePositionTrajectoryRefCommand drone_tra_cmd;

    droneMsgsROS::dronePositionRefCommand drone_ref_cmd;

    double dist_deone_to_end = sqrt(pow((end.position.x-point_current_drone.x), 2) + pow((end.position.y-point_current_drone.y), 2));


    if(replan == true)
    {

      position_pub = true;
    }


    if(replan == true || path_plan_start == true)
    {
       drone_tra_cmd.droneTrajectory.clear();
       for(int i = 0; i +3 < traj_num; i = i+3)
       {
           drone_ref_cmd.x = myTrajectory.poses[i].pose.position.x;
           drone_ref_cmd.y = myTrajectory.poses[i].pose.position.y;
           drone_ref_cmd.z = kUav_Altitude_;
           drone_tra_cmd.droneTrajectory.push_back(drone_ref_cmd);
       }
       if(position_pub == true && dist_deone_to_end > 0.1)
       {
        std::cout<<"coming !!"<<std::endl;
       traj_control_pub.publish(drone_tra_cmd);
       position_pub = false;
       }

    }

}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

void LASER_BASED_RRT_STAR::rviz_pathCallBack(const geometry_msgs::PointStamped& msg)
{
    geometry_msgs::Point p;
    p.x=msg.point.x;
    p.y=msg.point.y;
    p.z=msg.point.z;
    cout<<"choose point...please wait...."<<endl;
    points.points.push_back(p);


}

void LASER_BASED_RRT_STAR::Publish_P()
{

    geometry_msgs::Point p;
    points.header.frame_id="map";
    points.ns="markers";
    points.id = 6;
   points.header.stamp=ros::Time(0);
   points.type = points.POINTS;
   points.action =points.ADD;
   points.pose.orientation.w =1.0;

   points.scale.x=0.3;
   points.scale.y=0.3;

   points.color.r = 255.0/255.0;
   points.color.g = 0.0/255.0;
   points.color.b = 0.0/255.0;
   points.color.a=1.0;
   points.lifetime = ros::Duration();
    while(points.points.size()<5)
    {

      rrt_star_publisher.publish(points);

      ros::spinOnce();

    }
    std::cout<<"coming in point !!!"<<std::endl;
    std::cout<<"bound_x0="<<points.points[0].x<<std::endl;
    std::cout<<"bound_y0="<<points.points[0].y<<std::endl;
    std::cout<<"bound_x2="<<points.points[2].x<<std::endl;
    std::cout<<"bound_y2="<<points.points[2].y<<std::endl;
    std::cout<<"go!!!"<<std::endl;

    std::vector<float> temp1;
    temp1.push_back(points.points[0].x);
    temp1.push_back(points.points[0].y);

    std::vector<float> temp2;
    temp2.push_back(points.points[2].x);
    temp2.push_back(points.points[0].y);


    init_map_x=RRTNorm(temp1,temp2);
    temp1.clear();		temp2.clear();

    temp1.push_back(points.points[0].x);
    temp1.push_back(points.points[0].y);

    temp2.push_back(points.points[0].x);
    temp2.push_back(points.points[2].y);

    init_map_y=RRTNorm(temp1,temp2);
    temp1.clear();		temp2.clear();

    Xstartx=(points.points[0].x+points.points[2].x)*.5;
    Xstarty=(points.points[0].y+points.points[2].y)*.5;
    points.points.clear();
    rrt_star_publisher.publish(points);
    std::cout<<"point choose is ok!"<<std::endl;
    begin = ros::Time::now();
}

//Norm function
float LASER_BASED_RRT_STAR::RRTNorm(std::vector<float> x1,std::vector<float> x2)
{
return pow(	(pow(abs((x2[0]-x1[0])),2)+pow(abs((x2[1]-x1[1])),2))	,0.5);
}

//sign function
float LASER_BASED_RRT_STAR::RRTsign(float n)
{
if (n<0.0){return -1.0;}
else{return 1.0;}
}

//Steer function
std::vector<float> LASER_BASED_RRT_STAR::RRTSteer(  std::vector<float> x_nearest , std::vector<float> x_rand, float eta)
{
std::vector<float> x_new;

if (RRTNorm(x_nearest,x_rand)<=eta){
x_new=x_rand;
}
else{


float m=(x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0]);

x_new.push_back(  (RRTsign(x_rand[0]-x_nearest[0]))* (   sqrt( (pow(eta,2)) / ((pow(m,2))+1) )   )+x_nearest[0] );
x_new.push_back(  m*(x_new[0]-x_nearest[0])+x_nearest[1] );

if(x_rand[0]==x_nearest[0]){
x_new[0]=x_nearest[0];
x_new[1]=x_nearest[1]+eta;
}



}
return x_new;
}

bool LASER_BASED_RRT_STAR::checkCollision(Vector2f &start, Vector2f &end, double eta)
{

    std::vector<float> xnear;
    std::vector<float> xnew;
    double p1_x = start.x();
    double p1_y = start.y();
    double p2_x = end.x();
    double p2_y = end.y();
    xnear.push_back(p1_x);
    xnear.push_back(p1_y);
    xnew.push_back(p2_x);
    xnew.push_back(p2_y);

    float dist = Get_Dist_fromSDF(xnear[0],xnear[1]);
    if (dist > eta)
    {
        return true;
    }

    std::vector<float> new_point_;
    new_point_ = xnear;
    char obs = 0;


    while(dist<eta+0.06)
    {
    new_point_ = RRTSteer(new_point_,xnew,dist);
    dist = Get_Dist_fromSDF(new_point_[0],new_point_[1]);
    eta = RRTNorm(new_point_,xnew);
    if(dist < 0.06) {     obs=1; break;}

    }
    if(obs == 1)
    {
        return false;
    }
    else
    {
        return true;
    }

}

bool LASER_BASED_RRT_STAR::checkSample(Vector2f &n)
{
 float dist = Get_Dist_fromSDF(n.x(),n.y());
 if (dist >= 0)
   {
     return true;
 }
 else
   {
     return false;
 }
}

void LASER_BASED_RRT_STAR::rrt_star_perform(geometry_msgs::Pose startpoint,geometry_msgs::Pose endpoint)
{

    RRTSTAR rrtstar;

    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker rrtTreeMarker1;
    visualization_msgs::Marker rrtTreeMarker2;
    visualization_msgs::Marker finalPath;
    visualization_msgs::Marker finalsmoothPath;
    //if(uav_now_pos.x != NULL && singledrone_point.x != NULL)
  //{
    initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker1, rrtTreeMarker2,finalPath,finalsmoothPath);
    //setting source and goal
    //--------------test---------------------//
    cout<<"start_x = "<<startpoint.position.x<<endl;
    cout<<"start_y = "<<startpoint.position.y<<endl;
    cout<<"goal_x = "<<endpoint.position.x<<endl;
    cout<<"goal_y = "<<endpoint.position.y<<endl;
    //######################rrt_star_path#################//
    //get step size and max iterations.
     rrtstar.setMaxIterations(50);
     rrtstar.setStepSize(1);
     rrtstar.setSTART(startpoint.position.x,startpoint.position.y);
     rrtstar.setEND(endpoint.position.x,endpoint.position.y);
     srand (time(NULL));
    // RRTSTAR Algorithm

    for(int i = 0; i <  rrtstar.max_iter; i++) {
        Node* q;
        float x1 = (rand()%1000 /1000.0 *init_map_x)-(init_map_x*0.5)+Xstartx;
        float y1= (rand()%1000 /1000.0*init_map_y)-(init_map_y*0.5)+Xstarty;
        Vector2f point(x1, y1);
        float orient = rand()%1000 /1000.0 * 2 * 3.142;

        if (point.x() >= -51 && point.x() <= 51 && point.y() >= -51 && point.y() <= 51 && orient > 0 && orient < 2*3.142) {
            q = new Node;
            q->position = point;
            q->orientation = orient;
        }
        else
        {
            q=NULL;
        }

        if ( rrtstar.reached() && checkCollision(rrtstar.lastNode->position, rrtstar.endPos,rrtstar.step_size)) {
           cout<<"Reached Destination!"<<endl;
            break;
        }
        if (q) {

            Node *qNearest =  rrtstar.nearest(q->position);
            if ( rrtstar.distance(q->position, qNearest->position) >  rrtstar.step_size) {
                Vector3f newConfigPosOrient;

                newConfigPosOrient = rrtstar.newConfig(q, qNearest);
                Vector2f newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y());
                if (checkCollision(newConfigPos,qNearest->position,rrtstar.step_size)) {

                    Node *qNew = new Node;
                    qNew->position = newConfigPos;
                    qNew->orientation = newConfigPosOrient.z();
                    qNew->path = path;
                    vector<Node *> Qnear;
                     rrtstar.near(qNew->position,  rrtstar.step_size*RRTSTAR_NEIGHBOR_FACTOR, Qnear);
                    Node *qMin = qNearest;
                    double cmin =  rrtstar.Cost(qNearest) +  rrtstar.PathCost(qNearest, qNew);
                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(checkCollision(qNear->position, qNew->position,rrtstar.step_size) &&
                                ( rrtstar.Cost(qNear)+rrtstar.PathCost(qNear, qNew)) < cmin ){
                            qMin = qNear; cmin =  rrtstar.Cost(qNear)+ rrtstar.PathCost(qNear, qNew);
                        }
                    }
                     rrtstar.add(qMin, qNew);
                     addBranchtoRRTTree(rrtTreeMarker1,qMin,qNew);

                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(checkCollision(qNew->position, qNear->position,rrtstar.step_size) &&
                                ( rrtstar.Cost(qNew)+ rrtstar.PathCost(qNew, qNear)) <  rrtstar.Cost(qNear) ){
                            Node *qParent = qNear->parent;
                            // Remove edge between qParent and qNear
                            qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());
                            // Add edge between qNew and qNear
                            qNear->cost =  rrtstar.Cost(qNew) +  rrtstar.PathCost(qNew, qNear);
                            qNear->parent = qNew;
                            qNew->children.push_back(qNear);

                        }
                    }

                }


            }
        }

    }
    Node *q;
    if ( rrtstar.reached()) {
        q =  rrtstar.lastNode;

    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q =  rrtstar.nearest(rrtstar.endPos);
        cout<<"Exceeded max iterations!"<<endl;

    }
    // generate shortest path to destination.
    while (q != NULL) {
        rrtstar.path.push_back(q);
        q = q->parent;
    }
    if(rrtstar.path.size()>0)
    {
        if(rrtstar.path.size() > 3)
         {
          Path_Short(rrtstar,rrtstar.endPos.x(),rrtstar.endPos.y(),rrtstar.startPos.x(),rrtstar.startPos.y());
         }
          setFinalPathData(rrtstar.path,finalPath,rrtstar.endPos.x(),rrtstar.endPos.y());
          Path_Smooth(rrtstar.path,finalsmoothPath,rrtstar.endPos.x(),rrtstar.endPos.y(),rrtstar.startPos.x(),rrtstar.startPos.y());
          rrt_star_publisher.publish(finalPath);
          rrt_smooth_marker_pub.publish(finalsmoothPath);
    }


}

//------------------------------------------------------------------algorithm tests----------------------------------------------------------------------------------


void LASER_BASED_RRT_STAR::Reset()
{


    cv::Point2f uav_rand_pos, target_rand_pos;
    GenerateRandomPositions( uav_rand_pos, target_rand_pos);
    gazebo_msgs::SetModelState model_msg_uav;
    model_msg_uav.request.model_state.model_name = UAV_NAME;
    model_msg_uav.request.model_state.pose.position.x = uav_rand_pos.x;
    model_msg_uav.request.model_state.pose.position.y = uav_rand_pos.y;
    model_msg_uav.request.model_state.pose.position.z = kUav_Altitude_;
    if (gazebo_set_model_state_srv_.call(model_msg_uav))
    {

    }
    else{
        ROS_ERROR("ENV_INFO: Failed to call set model state");
//        return false;
    }

    gazebo_msgs::SetModelState model_msg_target;

    model_msg_target.request.model_state.model_name = TARGET_NAME;
    model_msg_target.request.model_state.pose.position.x = target_rand_pos.x;
    model_msg_target.request.model_state.pose.position.y = target_rand_pos.y;
    model_msg_target.request.model_state.pose.position.z = 0.0;
    if (gazebo_set_model_state_srv_.call(model_msg_target))
    {

    }
    else{
        ROS_ERROR("ENV_INFO: Failed to call set model state");
//        return false;
    }

    // Spawn model to origin
    std_srvs::Empty estimator_msg;

    if (estimator_client_.call(estimator_msg))
    {
        std_msgs::String reset_msg;

        reset_msg.data = "reset";

        hector_mapping_reset_pub_.publish(reset_msg);

        uav_initial_reset_position_.x = uav_rand_pos.x;

        uav_initial_reset_position_.y = uav_rand_pos.y;
        uav_initial_reset_position_.z = kUav_Altitude_;

        ROS_INFO("ENV_INFO: Reseting estimator..");

    }
    else{
        ROS_ERROR("ENV_INFO: Failed to call estimator");

    }


}

void LASER_BASED_RRT_STAR::GenerateRandomPositions(cv::Point2f &uav_pose, cv::Point2f &target_pose)
{

    std::srand (static_cast <unsigned> (time(0)));

    float max_x_ = 6.0;
    float max_y_ = 6.0;

//    float max_pos_x = (environment_info_.max_pos_x_ / 2.0) - 1;
//    float max_pos_y = (environment_info_.max_pos_y_ / 2.0) - 1;

    float max_pos_x = (max_x_ / 2.0) - 1;
    float max_pos_y = (max_y_ / 2.0) - 1;


         //*********** FOR TESTING PURPOSES ***********
        float lower_lim_x = -max_pos_x;
        float upper_lim_x = max_pos_x;
        float lower_lim_y = -1.0;
        float upper_lim_y = 1.0;
        float rand_pos_x = -3.0;
        float rand_pos_y = lower_lim_y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y-lower_lim_y)));
        uav_pose.x = rand_pos_x;
        uav_pose.y = rand_pos_y;

        lower_lim_x = -max_pos_x;
        upper_lim_x = max_pos_x;
        lower_lim_y = -1;
        upper_lim_y = 1;

        lower_lim_y = 0.0;
        rand_pos_x = 2.0;
        rand_pos_y = lower_lim_y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y-lower_lim_y)));
        target_pose.x = rand_pos_x;
        target_pose.y = rand_pos_y;


}

void LASER_BASED_RRT_STAR::PoseVelocityCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // Get position of the uav in the vector;

    cv::Point3f uav_previous_pos, uav_previous_linear_vel;
    uav_previous_pos.x = uav_state_.pos_x_;uav_previous_pos.y = uav_state_.pos_y_;uav_previous_pos.z = uav_state_.pos_z_;
    uav_previous_linear_vel.x = uav_state_.speed_x_;uav_previous_linear_vel.y = uav_state_.speed_y_;uav_previous_linear_vel.z = uav_state_.speed_z_;

    for (int i=0; i<msg->name.size(); i++)
    {
        if (msg->name[i].compare(UAV_NAME) == 0)
        {


            uav_position.x = msg->pose[i].position.x;
            uav_position.y = msg->pose[i].position.y;
            uav_position.z = msg->pose[i].position.z;

            uav_linear_vel.x = msg->twist[i].linear.x;
            uav_linear_vel.y = msg->twist[i].linear.y;
            uav_linear_vel.z = msg->twist[i].linear.z;


            break;


        }

        else if(msg->name[i].compare(TARGET_NAME) == 0)
        {
            target_position.x = msg->pose[i].position.x;
            target_position.y = msg->pose[i].position.y;
            target_position.z = msg->pose[i].position.z;
        }
        else if(msg->name[i].compare("moving_obstacle_squared1") == 0)
        {
            moving_obstacle_squared_.pos_x_ = msg->pose[i].position.x;
            moving_obstacle_squared_.pos_y_ = msg->pose[i].position.y;
            moving_obstacle_squared_.pos_z_ = msg->pose[i].position.z;
        }
    }



//---------------------------------------------------------------------------------------------------------------------------
        if((uav_position.x > -1.4 && uav_position.x < -1.36) || (uav_position.x > -0.3 && uav_position.x < -0.25))
        {
            gazebo_msgs::SetModelState model_msg_obstacle_cylinder;
            model_msg_obstacle_cylinder.request.model_state.model_name = "moving_obstacle_squared1";
            model_msg_obstacle_cylinder.request.model_state.pose.position.x = uav_position.x + 1.0;
            model_msg_obstacle_cylinder.request.model_state.pose.position.y = uav_position.y;
            model_msg_obstacle_cylinder.request.model_state.pose.position.z = 0.0;
            if (gazebo_set_model_state_srv_.call(model_msg_obstacle_cylinder))
            {

            }
            else{
                ROS_ERROR("ENV_INFO: Failed to call set model state");
            }
        }

//---------------------------------------------------------------------------------------------------------------------------
//    else if(TEST_MODE && environment_info_.name_ == "house_3")
//    {
//        if((uav_position.x > -2.2 && uav_position.x < -2.195))
//        {
//            float lower_lim_y = -0.3;
//            float upper_lim_y = 0.3;
//            float rand_pos_y = lower_lim_y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y-lower_lim_y)));

//            gazebo_msgs::SetModelState model_msg_obstacle_cylinder1;
//            model_msg_obstacle_cylinder1.request.model_state.model_name = "moving_obstacle_squared_big1";
//            model_msg_obstacle_cylinder1.request.model_state.pose.position.x = uav_position.x + 1.2;
//            model_msg_obstacle_cylinder1.request.model_state.pose.position.y = uav_position.y + rand_pos_y;
//            model_msg_obstacle_cylinder1.request.model_state.pose.position.z = 0.0;
//            if (gazebo_set_model_state_srv_.call(model_msg_obstacle_cylinder1))
//            {

//            }
//            else{
//                ROS_ERROR("RL_ENV_INFO: Failed to call set model state");
//            }


//            lower_lim_y = -1.0;
//            upper_lim_y = 1.0;
//            rand_pos_y = lower_lim_y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y-lower_lim_y)));


//            gazebo_msgs::SetModelState model_msg_obstacle_cylinder2;
//            model_msg_obstacle_cylinder2.request.model_state.model_name = "moving_obstacle_squared_big2";
//            model_msg_obstacle_cylinder2.request.model_state.pose.position.x = 0.75;
//            if(rand_pos_y > 0)
//                model_msg_obstacle_cylinder2.request.model_state.pose.position.y = 1.8;
//            else
//                model_msg_obstacle_cylinder2.request.model_state.pose.position.y = -1.8;
//            model_msg_obstacle_cylinder2.request.model_state.pose.position.z = 0.0;
//            if (gazebo_set_model_state_srv_.call(model_msg_obstacle_cylinder2))
//            {

//            }
//            else{
//                ROS_ERROR("RL_ENV_INFO: Failed to call set model state");
//            }


//    if(DEBUG_UAV_TARGET_POSITON)
//    {
//        std::cout<<"UAV position: "<<"x: "<<uav_position.x<<" ; "<<"y: "<<uav_position.y<<" ; "<<std::endl;
//        std::cout<<"TARGET position: "<<"x: "<<target_position.x<<" ; "<<"y: "<<target_position.y<<std::endl;
//    }


    // Set uav state

    SetUavState(uav_position.x, uav_position.y, uav_position.z, uav_linear_vel.x, uav_linear_vel.y, uav_linear_vel.z);
    SetUavPreviousState(uav_previous_pos.x, uav_previous_pos.y, uav_previous_pos.z,
                        uav_previous_linear_vel.x, uav_previous_linear_vel.y, uav_previous_linear_vel.z);
    SetTargetState(target_position.x, target_position.y, target_position.z);
//---------------------------------------------------------------------------------------------------------------------------------------------//
        geometry_msgs::Point current_drone_point;

        current_drone_point.x = drone_pos.position.x;
        current_drone_point.y = drone_pos.position.y;
        geometry_msgs::Pose p_init;
        geometry_msgs::Pose p_end;
        p_end.position.x = target_position.x+3;
        p_end.position.y = target_position.y;
        if(path_plan_start == true || path_replan == true)
         {
            p_init.position.x = drone_pos.position.x;
            p_init.position.y = drone_pos.position.y;
           clock_t startTime,endTime;
           startTime = clock();
           rrt_star_perform(p_init, p_end);
           endTime = clock();
           std::cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
           }


        if(rrt_star_path_smooth.poses.size()>0)
          {
            //drone_position_control(rrt_star_path_smooth,current_drone_point,p_end,path_replan);
            drone_tra_control(rrt_star_path_smooth,current_drone_point,p_end,path_replan);
            path_plan_start = false;
            path_replan = false;

        }

      if(rrt_star_path_smooth.poses.size() > 0)
       {
//        Vector2f last_path_point;

//        Vector2f next_path_point;

//        if((point_path_cur+2) < rrt_star_path_smooth.poses.size())
//         {
//            last_path_point.x() = rrt_star_path_smooth.poses[point_path_cur].pose.position.x;
//            last_path_point.y() = rrt_star_path_smooth.poses[point_path_cur].pose.position.y;

//            next_path_point.x() = rrt_star_path_smooth.poses[point_path_cur+2].pose.position.x;
//            next_path_point.y() = rrt_star_path_smooth.poses[point_path_cur+2].pose.position.y;

//            Eigen::Vector2f v = last_path_point - next_path_point;
//            double dist = sqrt(pow(v.x(), 2) + pow(v.y(), 2));


//            if(!checkCollision(last_path_point,next_path_point,dist))
//              {
//                std::cout<<"in !"<<std::endl;

//                path_replan = true;
//                rrt_star_path_smooth.poses.clear();
//                rrt_star_path.poses.clear();

//            }
//          }

          int traj_rrt_num = rrt_star_path.poses.size();

          Eigen::Vector2f next_path_point;
          Eigen::Vector2f current_position;
          current_position.x() = drone_pos.position.x;
          current_position.y() =drone_pos.position.y;


          next_path_point.x() = rrt_star_path.poses[traj_rrt_num-point_near].pose.position.x;
          next_path_point.y() = rrt_star_path.poses[traj_rrt_num-point_near].pose.position.y;

          Eigen::Vector2f v = current_position - next_path_point;
          double dist = sqrt(pow(v.x(), 2) + pow(v.y(), 2));
          if(!checkCollision(current_position,next_path_point,dist))
          {
              path_replan = true;
              point_near = 2;
              rrt_star_path_smooth.poses.clear();
              rrt_star_path.poses.clear();

          }
          if(dist < 0.05 && point_near <= traj_rrt_num)
          {
            point_near = point_near+1;
          }

      }

    float delta_x =  p_end.position.x  - drone_pos.position.x;
    float delta_y = p_end.position.y - drone_pos.position.y;
    float relative_pos = std::sqrt(std::pow(delta_x,2) + std::pow(delta_y, 2));
    if(relative_pos < 0.1)
    {
       goal_reached_ = 1;

    }

    //float min_range_t = *std::min_element(laser_info_.laser_ranges_.begin(), laser_info_.laser_ranges_.end());
       if(begin.toSec() >0 )
       {
        f_data_recorder<<ros::Time::now()-begin<<"\t"<<num_episode_<<"\t"<<goal_reached_<<"\t"<<
                         uav_position.x<<"\t"<<uav_position.y<<"\t"<<
                      p_end.position.x<<"\t"<<p_end.position.y<<"\t"<<
                         moving_obstacle_squared_.pos_x_<<"\t"<<moving_obstacle_squared_.pos_y_<<"\t"<<
                         drone_speed.dx<<"\t"<<drone_speed.dy<<"\t"<<min_range_t<<std::endl;
       }
//    if(relative_pos < 0.1)
//    {

//        Reset();
//        num_episode_ = num_episode_ + 1;
//        goal_reached_ = 0;

//        path_plan_start = true;
//        path_replan = false;
//        point_path_cur = 0;
//        position_pub = true;

//        rrt_star_path_smooth.poses.clear();
//        rrt_star_path.poses.clear();

//    }



}



void LASER_BASED_RRT_STAR::SetUavState(const float x, const float y, const float z,
                                                  const float dx, const float dy, const float dz)
{
    uav_mutex_.lock();
    uav_state_.pos_x_ = x;
    uav_state_.pos_y_ = y;
    uav_state_.pos_z_ = z;
    uav_state_.speed_x_ = dx;
    uav_state_.speed_y_ = dy;
    uav_state_.speed_z_ = dz;
    uav_mutex_.unlock();
}

void LASER_BASED_RRT_STAR::SetUavPreviousState(const float x, const float y, const float z,
                                                  const float dx, const float dy, const float dz)
{
    uav_mutex_.lock();
    uav_state_.prev_pos_x_ = x;
    uav_state_.prev_pos_y_ = y;
    uav_state_.prev_pos_z_ = z;
    uav_state_.prev_speed_x_ = dx;
    uav_state_.prev_speed_y_ = dy;
    uav_state_.prev_speed_z_ = dz;
    uav_mutex_.unlock();
}


void LASER_BASED_RRT_STAR::SetTargetState(const float x, const float y, const float z)
{
    target_mutex_.lock();
    target_state_.pos_x_ = x;
    target_state_.pos_y_ = y;
    target_state_.pos_z_ = z;
    target_mutex_.unlock();
}
