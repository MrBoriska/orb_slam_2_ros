#include "MonoNodeVI.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoVI");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);
    

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);

    return 0;
}


MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &nh, image_transport::ImageTransport &image_transport) : Node (sensor, nh, image_transport) {
  //image_subscriber = image_transport.subscribe ("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";

  Init();

  std::string name_of_node = ros::this_node::getName();

  // additional VI settings
  std::string settings_file;
  nh.param<std::string>(name_of_node + "/addl_settings_file", settings_file, "file_not_set");
  //ORB_SLAM2::ConfigParam config(settings_file.c_str());
  ORB_SLAM2::ConfigParam* pParams = new ORB_SLAM2::ConfigParam(settings_file.c_str());
  orb_slam_->SetConfigParam(pParams);


  //    if(config.GetRunningMode() == 1)
  //        SLAM.SetMonoVIEnable(true);   // set mbMonoVI=true, use MonoVI,
  //    if(config.GetDeactiveLoopClosure() == true)
  //        SLAM.SetDeactiveLoopCloserInMonoVI(true);   // set deactive loop closure detection

  if(pParams->GetRunningMode() == 1)
      orb_slam_->SetMonoVIEnable(true);   // set mbMonoVI=true, use MonoVI,
  if(pParams->GetDeactiveLoopClosure() == true)
      orb_slam_->SetDeactiveLoopCloserInMonoVI(true);   // set deactive loop closure detection

  /**
   * @brief added data sync
   */
  // double imageMsgDelaySec = config.GetImageDelayToIMU();
  double imageMsgDelaySec = pParams->GetImageDelayToIMU();
  ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
  /// Step 2: Subscribing the image and imu topics, as well as recalling the response function to record data
  ros::Subscriber imagesub = nh.subscribe("/camera/image_raw", 3, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
  ros::Subscriber imusub = nh.subscribe("/imu", 150, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);

  SensorMsgImagePtr imageMsg;
  std::vector<SensorMsgImuPtr> vimuMsg;

  // 3dm imu output per g. 1g=9.80665 according to datasheet
  //const double g3dm = 9.80665;
  //const bool bAccMultiply98 = config.GetAccMultiply9p8();     // default: 0
  //const bool bAccMultiply98 = pParams->GetAccMultiply9p8();     // default: 0

  ros::Rate r(30);
  int nImages = 0;    // image number
  while(ros::ok())
  {
    bool bdata = msgsync.getRecentMsgs(imageMsg, vimuMsg);  // get image and its all imu datas collected from last image
    if (bdata)  // subscribe a new image
    {
      nImages ++;

      // reconstruct the std::vector<SensorMsgImuPtr> into std::vector<ORB_SLAM2::IMUData>
      std::vector<ORB_SLAM2::IMUData> vimuData;
      for (size_t i=0, iend=vimuMsg.size(); i<iend; i++)
      {
        SensorMsgImuPtr imuMsg = vimuMsg[i];
        double wx = imuMsg->angular_velocity.x;
        double wy = imuMsg->angular_velocity.y;
        double wz = imuMsg->angular_velocity.z;
        double ax = imuMsg->linear_acceleration.x;
        double ay = imuMsg->linear_acceleration.y;
        double az = imuMsg->linear_acceleration.z;
        double timu = imuMsg->header.stamp.toSec();
        
        //ax /= 10.1;
        //ay /= 10.1;
        //az /= 10.1;

        ORB_SLAM2::IMUData imudata( wx, wy, wz, ax, ay, az, timu );
        vimuData.push_back( imudata );
      }

      /// grab image and track
      GrabImage(imageMsg, vimuData);
    }

    ros::spinOnce();
    r.sleep();
  }

  // Save camera trajectory
  //orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  // Save NavState trajectory (IMU)
  //orb_slam_->SaveKeyFrameTrajectoryNavState("KeyFrameNavStateTrajectory.txt");

  // Stop all threads
  //orb_slam_->Shutdown();

  ros::shutdown();
}


MonoNode::~MonoNode () {
  
}

void MonoNode::GrabImage(const sensor_msgs::ImageConstPtr& msg, const std::vector<ORB_SLAM2::IMUData> vimuData)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    current_frame_time_ = msg->header.stamp;

    orb_slam_->TrackMonoVI(cv_ptr->image, vimuData, cv_ptr->header.stamp.toSec());
    
    Update();
}
