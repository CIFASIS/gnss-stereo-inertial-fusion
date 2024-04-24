/**
* This file is part of GNSS-SI. GNSS-SI is based on ORB-SLAM3.
* Copyright (C) 2023 Javier Cremona, Ernesto Kofman and Taihú Pire, CIFASIS (CONICET-UNR).
* Copyright (C) 2023 Javier Civera, University of Zaragoza.
* This file was modified by Javier Cremona.
*
* Original authors:
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* GNSS-SI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GNSS-SI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with GNSS-SI.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/NavSatFix.h>
#include<geometry_msgs/TransformStamped.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"
#include"../include/GpsTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class GpsGrabber
{
public:
    GpsGrabber(bool bUseGps, double _noiseStdDev):mbUseGps(bUseGps), noiseStdDev(_noiseStdDev), gpsMeasId(0)
    {
        const double mean = 0.0;
        std::random_device rd;
        generator = std::mt19937(rd());
        noiseX = std::normal_distribution<float>(mean, noiseStdDev);
        noiseY = std::normal_distribution<float>(mean, noiseStdDev);
        noiseZ = std::normal_distribution<float>(mean, noiseStdDev);

    };
    void GrabGps(const sensor_msgs::NavSatFixConstPtr &gps_msg);
    void GrabVicon(const geometry_msgs::TransformStampedConstPtr &transform_msg);
    void SaveMeasurement(const ORB_SLAM3::GlobalPosition::GlobalPosition*);
    void ClearMeasurements();

    // TODO should we use a lock for mbUseGps?? In the future, we would like to modify this boolean in runtime
    bool mbUseGps;
    queue<const ORB_SLAM3::GlobalPosition::GlobalPosition*> gpsBuf;
    std::list<const ORB_SLAM3::GlobalPosition::GlobalPosition*> allMeasurements;
    std::mutex mBufMutex;    
    double noiseStdDev;
    int gpsMeasId;
    std::normal_distribution<float> noiseX;
    std::normal_distribution<float> noiseY;
    std::normal_distribution<float> noiseZ;
    std::mt19937 generator;
};


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, GpsGrabber *pGpsGb, const bool bRect, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mpGpsGb(pGpsGb), do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;
    GpsGrabber *mpGpsGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

bool useGps(const string &strSettingsFile)
{
  cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      cerr << "ERROR: Wrong path to settings" << endl;
      throw -1;
  }
  int useGlobalMeas = fsSettings["System.UseGlobalMeas"];
  return useGlobalMeas != 0;
}

float getNoiseStdDev(const string &strSettingsFile)
{
  cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      cerr << "ERROR: Wrong path to settings" << endl;
      throw -1;
  }
  float useGlobalMeas = fsSettings["System.GPSSimulatedNoise"];
  return useGlobalMeas;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 4 || argc > 5)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  std::string sbRect(argv[3]);
  if(argc==5)
  {
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);

  bool mbUseGps = useGps(argv[2]);
  float noiseStdDev = getNoiseStdDev(argv[2]);
  ImuGrabber imugb;
  GpsGrabber gpsgb(mbUseGps, noiseStdDev);
  ImageGrabber igb(&SLAM,&imugb,&gpsgb,sbRect == "true",bEqual);
  
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight,&igb);
  ros::Subscriber sub_gps = n.subscribe("/gps/fix", 1000, &GpsGrabber::GrabGps, &gpsgb); 
  ros::Subscriber sub_vicon = n.subscribe("/vicon/firefly_sbx/firefly_sbx", 1000, &GpsGrabber::GrabVicon, &gpsgb); 
  

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();
  if(!ros::ok())
  {
    std::cout << "Shutting down..." << std::endl;
    SLAM.forceSystemShutdown(false);
    gpsgb.ClearMeasurements();
  }

  return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRight.lock();
  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  std::cout << std::boolalpha;
  std::cout << "Using GPS: " << mpGpsGb->mbUseGps << std::endl;
  const double maxTimeDiff = 0.01;
  //int gpsMeasId = 0;
  while(1)
  {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      this->mBufMutexRight.lock();
      while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
      {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      vector<const ORB_SLAM3::GlobalPosition::GlobalPosition*> vGpsMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();

      mpGpsGb->mBufMutex.lock();
      // TODO should we use a lock for mbUseGps?? In the future, we would like to modify this boolean in runtime
      if(mpGpsGb->mbUseGps && !mpGpsGb->gpsBuf.empty())
      {
        vGpsMeas.clear();
        while(!mpGpsGb->gpsBuf.empty() && mpGpsGb->gpsBuf.front()->timestamp<=tImLeft)
        {          
          const ORB_SLAM3::GlobalPosition::GlobalPosition* meas = mpGpsGb->gpsBuf.front();
          //double t = meas->header.stamp.toSec();
          //std::cout << "MEASSS: " << meas->id << std::endl;
          vGpsMeas.push_back(meas);
          //gpsMeasId++;
          mpGpsGb->gpsBuf.pop();        
        }
      }
      mpGpsGb->mBufMutex.unlock();

      if(mbClahe)
      {
        mClahe->apply(imLeft,imLeft);
        mClahe->apply(imRight,imRight);
      }

      if(do_rectify)
      {
        cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
      }

      mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas,vGpsMeas);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

void GpsGrabber::GrabGps(const sensor_msgs::NavSatFixConstPtr &gps_msg)
{
  if(!mbUseGps)
    return;

  Eigen::Matrix3d covariance;
  const int POSITION_SIZE = 3;
  if(noiseStdDev == 0.0)
  {     
    for (size_t i = 0; i < POSITION_SIZE; i++) 
    {
      for (size_t j = 0; j < POSITION_SIZE; j++) 
      {
        covariance(i, j) =
          gps_msg->position_covariance[POSITION_SIZE * i + j];
      }
    }
    ORB_SLAM3::GlobalPosition::GpsMeasurement *g = new ORB_SLAM3::GlobalPosition::GpsMeasurement(gpsMeasId, gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, gps_msg->header.stamp.toSec(), covariance);
    SaveMeasurement(g);
  }
  else if(noiseStdDev > 0.0)
  { 
    covariance.setZero();
    for (size_t i = 0; i < POSITION_SIZE; i++)
    {
      covariance(i,i) = noiseStdDev * noiseStdDev;
    }
    const float nx = noiseX(generator);
    const float ny = noiseY(generator);
    const float nz = noiseZ(generator);
    Eigen::Vector3d noise(nx,ny,nz);
    ORB_SLAM3::GlobalPosition::GpsMeasurement *g = new ORB_SLAM3::GlobalPosition::GpsMeasurement(
      gpsMeasId, 
      gps_msg->latitude, 
      gps_msg->longitude, 
      gps_msg->altitude, 
      gps_msg->header.stamp.toSec(),
      covariance,
      noise);
    SaveMeasurement(g);
  }
  //std::cout << g.id << std::endl;
  gpsMeasId++;
  return;
}

void GpsGrabber::GrabVicon(const geometry_msgs::TransformStampedConstPtr &transform_msg)
{
  if(!mbUseGps)
    return;

  const int POSITION_SIZE = 3;
  if(noiseStdDev == 0.0)
  {     
    throw std::logic_error("Function not yet implemented");  
  }
  else if(noiseStdDev > 0.0)
  { 
    Eigen::Matrix3d covariance;
    covariance.setZero();
    for (size_t i = 0; i < POSITION_SIZE; i++)
    {
      covariance(i,i) = noiseStdDev * noiseStdDev;
    }
    const float nx = noiseX(generator);
    const float ny = noiseY(generator);
    const float nz = noiseZ(generator);
    Eigen::Vector3d noise(nx,ny,nz);    
    ORB_SLAM3::GlobalPosition::MocapMeasurement *p = new ORB_SLAM3::GlobalPosition::MocapMeasurement(
    gpsMeasId, 
    transform_msg->transform.translation.x, 
    transform_msg->transform.translation.y, 
    transform_msg->transform.translation.z, 
    transform_msg->transform.rotation.x,
    transform_msg->transform.rotation.y,
    transform_msg->transform.rotation.z,
    transform_msg->transform.rotation.w,
    transform_msg->header.stamp.toSec(),
    covariance,
    noise);
    SaveMeasurement(p);
  }
  gpsMeasId++;
  return;
}

void GpsGrabber::SaveMeasurement(const ORB_SLAM3::GlobalPosition::GlobalPosition* m)
{
  allMeasurements.push_back(m);
  #ifdef GPS_DROPOUT
    if((gpsMeasId > 300 && gpsMeasId < 350) || (gpsMeasId > 500 && gpsMeasId < 650))
    { 
      std::cout << "Discarded: " << gpsMeasId << std::endl;
      return;
    }
        
  #endif

  mBufMutex.lock();
  gpsBuf.push(m);  
  mBufMutex.unlock();
}

void GpsGrabber::ClearMeasurements()
{
  for(auto m : allMeasurements)
  {
    delete m;
  }
  
}
