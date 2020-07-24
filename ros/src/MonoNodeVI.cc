#include "MonoNodeVI.h"

using namespace std;

void SaveTimestamps(const vector<double> vTimestamps, const string &filename);

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

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vector<double> vImageTimestamps;
  vector<double> vImuTimestamps;

  vector<double> vTimesOfProcessingFrame;
  vector<double> vTimesOfTrack;
  vector<double> vTimesOfConstructFrame;
  vector<double> vTimesOfTrackWithIMU;
  vector<double> vTimesOfTrackLocalMapWithIMU;

  vector<double> vTimesOfComputePyramid;
  vector<double> vTimesOfComputeKeyPointsOctTree;
  vector<double> vTimesOfComputeDescriptor;

  /**
   * @brief added data sync
   */
  // double imageMsgDelaySec = config.GetImageDelayToIMU();
  double imageMsgDelaySec = pParams->GetImageDelayToIMU();
  ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
  /// Step 2: Subscribing the image and imu topics, as well as recalling the response function to record data
  ros::Subscriber imagesub = nh.subscribe("/camera/image_raw", 3, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
  ros::Subscriber imusub = nh.subscribe("/imu", 300, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);

  SensorMsgImagePtr imageMsg;
  std::vector<SensorMsgImuPtr> vimuMsg;

  // 3dm imu output per g. 1g=9.80665 according to datasheet
  const double g3dm = 9.80665;
  //const bool bAccMultiply98 = config.GetAccMultiply9p8();     // default: 0
  const bool bAccMultiply98 = pParams->GetAccMultiply9p8();     // default: 0

  ros::Rate r(400);
  Timer tTimeOfNoImage;
  bool bCheckTimeOfNoImage = false;
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
        vImuTimestamps.push_back(timu);
        if(bAccMultiply98)
        {
            ax *= g3dm;
            ay *= g3dm;
            az *= g3dm;
        }
        ORB_SLAM2::IMUData imudata( wx, wy, wz, ax, ay, az, timu );
        vimuData.push_back( imudata );
      }

      double tframe = imageMsg->header.stamp.toSec();
      vImageTimestamps.push_back( tframe );    // record stamps of new image

      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      /// grab image and track
      GrabImage(imageMsg, vimuData);

      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000; // ms

      // Time statistic: Only record the time of successful tracking frame using vision & IMU.
      //                 Ignore the time-consuming of VI-ORB_SLAM initialization section since it uses only vision.
      if(orb_slam_->GetTrackWithIMUState())
      {
          // std::cout << "record time " << std::endl;
          vTimesTrack.push_back(ttrack);      // record track time of new image

          vTimesOfProcessingFrame.push_back(orb_slam_->GetTimeOfProcessingFrame());
          vTimesOfTrack.push_back(orb_slam_->GetTimeOfTrack());
          vTimesOfConstructFrame.push_back(orb_slam_->GetTimeOfConstructFrame());
          vTimesOfTrackWithIMU.push_back(orb_slam_->GetTimeOfTrackWithIMU());
          vTimesOfTrackLocalMapWithIMU.push_back(orb_slam_->GetTimeOfTrackLocalMapWithIMU());

          vTimesOfComputePyramid.push_back(orb_slam_->GetTimeOfComputePyramid());
          vTimesOfComputeKeyPointsOctTree.push_back(orb_slam_->GetTimeOfComputeKeyPointsOctTree());
          vTimesOfComputeDescriptor.push_back(orb_slam_->GetTImeOfComputeDescriptor());
      }

      bCheckTimeOfNoImage = true;
      tTimeOfNoImage.freshTimer(); // refresh timer
    }

    // Stop system: if more than 10s the system do not receive any image
    if( tTimeOfNoImage.runTime_s() > 10 && bCheckTimeOfNoImage )
    {
      break;
    }


    ros::spinOnce();
    r.sleep();
  }

  // Stop all threads
  // orb_slam_->Shutdown();

  double totaltime = 0;

  double totalTimeOfProcessingFrame = 0;
  double totalTimeOfTrack = 0;
  double totalTimeOfConstructFrame = 0;
  double totalTimeOfTrackWithIMU = 0;
  double totalTimeOfTrackLocalMapWithIMU = 0;

  double totalTimeOfComputePyramid = 0;
  double totalTimeOfComputeKeyPointsOctTree = 0;
  double totalTimeOfComputeDescriptor = 0;

  int timeRecordNum = vTimesOfTrack.size();

  for(int ni=0; ni<timeRecordNum; ni++) // nImages
  {
      totaltime+=vTimesTrack[ni];

      totalTimeOfProcessingFrame += vTimesOfProcessingFrame[ni];
      totalTimeOfTrack += vTimesOfTrack[ni];
      totalTimeOfConstructFrame += vTimesOfConstructFrame[ni];
      totalTimeOfTrackWithIMU += vTimesOfTrackWithIMU[ni];
      totalTimeOfTrackLocalMapWithIMU += vTimesOfTrackLocalMapWithIMU[ni];

      totalTimeOfComputePyramid += vTimesOfComputePyramid[ni];
      totalTimeOfComputeKeyPointsOctTree += vTimesOfComputeKeyPointsOctTree[ni];
      totalTimeOfComputeDescriptor += vTimesOfComputeDescriptor[ni];

  }
  cout << endl << "-------" << endl;
  cout << "Time statistic: Only record the time of successful tracking frame using vision & IMU." << endl;
  cout << "                Ignore the time-consuming of VI-ORB_SLAM initialization section since it uses only vision." << endl;
  //cout << "median tracking time: " << vTimesTrack[timeRecordNum/2] << " ms"<< endl;
  //cout << "mean tracking time: " << totaltime/timeRecordNum << " ms" << endl;

  cout << "| mean time of ProcessingFrame:              " << totalTimeOfProcessingFrame           / timeRecordNum << " ms" << endl;
  cout << "   | mean time of ConstructFrame:              " << totalTimeOfConstructFrame            / timeRecordNum << " ms" << endl;
  cout << "       | mean time of ComputePyramid:              " << totalTimeOfComputePyramid            / timeRecordNum << " ms" << endl;
  cout << "       | mean time of ComputeKeyPointsOctTre:      " << totalTimeOfComputeKeyPointsOctTree   / timeRecordNum << " ms" << endl;
  cout << "       | mean time of ComputeDescriptor:           " << totalTimeOfComputeDescriptor         / timeRecordNum << " ms" << endl;
  cout << "   | mean time of Track:                       " << totalTimeOfTrack                     / timeRecordNum << " ms" << endl;
  cout << "       | mean time of TrackWithIMU:                " << totalTimeOfTrackWithIMU              / timeRecordNum << " ms" << endl;
  cout << "       | mean time of TrackLocalMapWithIMU:        " << totalTimeOfTrackLocalMapWithIMU      / timeRecordNum << " ms" << endl;

  // Save camera trajectory
  //orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  // Save NavState trajectory (IMU)
  orb_slam_->SaveKeyFrameTrajectoryNavState("KeyFrameNavStateTrajectory.txt");

  SaveTimestamps(vImageTimestamps, "ImageTimestamps.txt");
  SaveTimestamps(vImuTimestamps, "ImuTimestamps.txt");

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

void SaveTimestamps(const vector<double> vTimestamps, const string &filename)
{
    ofstream f;
    f.open( filename.c_str() );
    f << fixed;

    for (size_t i=0, iend=vTimestamps.size(); i<iend; i++)
    {
        f << setprecision(6) << vTimestamps[i] << endl;
    }
    f.close();
    std::cout << std::endl << " timestamps saved in file:  " << filename << std::endl;
}
