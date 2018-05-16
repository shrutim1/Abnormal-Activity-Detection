// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "PersonTrackingSample.h"
#include <iostream>
#include <fstream>
#include "realsense_ros_person/TrackingConfig.h"
#include "realsense_ros_person/Recognition.h"
#include "realsense_ros_person/StartTracking.h"
#include "realsense_ros_person/StopTracking.h"
#include "realsense_ros_person/RecognitionRegister.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <realsense_ros_person/PersonModuleState.h>
#include <string>
#include <math.h>
#include <cmath>
#include <sys/time.h>
#include <stdio.h>
//#include "sound_play.h"
#include </home/shruti/catkin_ws/src/realsense_samples_ros/realsense_ros_person/src/sample/sound_play/include/sound_play/sound_play.h>
#include <ctime>
int framec = 0;
std::ofstream myfile;
std::string PersonTrackingSample::PERSON_MODULE_STATE_TOPIC = "/person_tracking/module_state";
std::string RegistrationResultToString(int status);
std::string RecognitionResultToString(int status);
std::string WaveGestureToString(int32_t waveGestureRos);
struct timeval tp;
int count =0,count1=0;
struct Skeleton{
	int x;
	int y;
	long int ti;
  };

PersonTrackingSample::PersonTrackingSample() : m_viewer(false), m_trackingRenderer(m_viewer)
{
  m_trackingRenderer.SetPersonSelectionHandler([this](PersonData & data, TrackingRenderer::SelectType type)
  {
    PersonSelectedHandler(data, type);
  });
  m_trackingRenderer.SetGlobalHandler([this](TrackingRenderer::SelectType type)
  {
    GlobalHandler(type);
  });
}

void PersonTrackingSample::ProcessCommandLineArgs()
{
  ros::NodeHandle nodeHandle("~");
  nodeHandle.param<bool>("skeletonEnabled", mEnableSkeleton, true); //false
  ROS_INFO_STREAM("mEnableSkeleton = " << mEnableSkeleton);

  nodeHandle.param<bool>("recognitionEnabled", mEnableRecognition, true);
  ROS_INFO_STREAM("mEnableRecognition = " << mEnableRecognition);

  nodeHandle.param<bool>("pointingGestureEnabled", mEnablePointingGesture, false);
  ROS_INFO_STREAM("mEnablePointingGesture = " << mEnablePointingGesture);

  nodeHandle.param<bool>("waveGestureEnabled", mEnableWaveGesture, false);
  ROS_INFO_STREAM("mEnableWaveGesture = " << mEnableWaveGesture);

  nodeHandle.param<bool>("landmarksEnabled", mEnableLandmarks, true);
  ROS_INFO_STREAM("mEnableLandmarks = " << mEnableLandmarks);

  nodeHandle.param<bool>("headBoundingBoxEnabled", mEnableHeadBoundingBox, true);
  ROS_INFO_STREAM("headBoundingBox = " << mEnableHeadBoundingBox);

  nodeHandle.param<bool>("headPoseEnabled", mEnableHeadPose, true);
  ROS_INFO_STREAM("headPose = " << mEnableHeadPose);
}

void PersonTrackingSample::InitMessaging(ros::NodeHandle& nodeHandle)
{
  EnableTrackingFeatures(nodeHandle);
  mTrackingOutputSubscriber = nodeHandle.subscribe("person_tracking_output_test", 1, &PersonTrackingSample::PersonTrackingCallback, this);
  mRecognitionRequestClient = nodeHandle.serviceClient<realsense_ros_person::Recognition>("person_tracking/recognition_request");
  mRegisterRequestClient = nodeHandle.serviceClient<realsense_ros_person::RecognitionRegister>("person_tracking/register_request");
  mStartTrackingRequestClient = nodeHandle.serviceClient<realsense_ros_person::StartTracking>("person_tracking/start_tracking_request");
  mStopTrackingRequestClient = nodeHandle.serviceClient<realsense_ros_person::StopTracking>("person_tracking/stop_tracking_request");
}

void PersonTrackingSample::EnableTrackingFeatures(ros::NodeHandle& nodeHandle)
{
  mConfigClient = nodeHandle.serviceClient<realsense_ros_person::TrackingConfig>("person_tracking/tracking_config");

  realsense_ros_person::TrackingConfig config;
  config.request.enableRecognition = mEnableRecognition;
  config.request.enableSkeleton = mEnableSkeleton;
  config.request.enablePointingGesture = mEnablePointingGesture;
  config.request.enableWaveGesture = mEnableWaveGesture;
  config.request.enableLandmarks = mEnableLandmarks;
  config.request.enableHeadBoundingBox = mEnableHeadBoundingBox;
  config.request.enableHeadPose = mEnableHeadPose;

  while (!mConfigClient.call(config))
  {
    //ROS_INFO_STREAM("failed to send config to person tracking node, sleep 1 second and retry");
    usleep(1e6);
  }
}

void PersonTrackingSample::PersonTrackingCallback(const realsense_ros_person::FrameTest& msg)
{
  // ROS_INFO_STREAM("Received person tracking output message. Number of people: " << msg.frameData.numberOfUsers);

  cv_bridge::CvImageConstPtr ptr;
  cv::Mat colorImage = cv_bridge::toCvShare(msg.colorImage, ptr, sensor_msgs::image_encodings::BGR8)->image;
  DrawDetectionResults(colorImage, msg);

  m_viewer.ShowImage(colorImage);
}

void PersonTrackingSample::DrawDetectionResults(cv::Mat& colorImage, const realsense_ros_person::FrameTest& msg)
{ 
  std::lock_guard<std::mutex> guard(mMutex);

  m_trackingRenderer.Reset();

  realsense_ros_person::Frame frame = msg.frameData;

  for (realsense_ros_person::User& user : frame.usersData)
  {
    DrawPersonResults(colorImage, user);
  }
}


void PersonTrackingSample::DrawPersonResults(cv::Mat& colorImage, realsense_ros_person::User& user)
{
  // person rectangle
  cv::Point pt1(user.userRect.rectCorners[0].x, user.userRect.rectCorners[0].y);
  cv::Point pt2(user.userRect.rectCorners[1].x, user.userRect.rectCorners[1].y);
  cv::Rect userRectangle(pt1, pt2);

  int personId = user.userInfo.Id;

  // center of mass point
  cv::Point centerMass(user.centerOfMassImage.x, user.centerOfMassImage.y);
  cv::Point3f centerMassWorld(user.centerOfMassWorld.x, user.centerOfMassWorld.y, user.centerOfMassWorld.z);

  m_trackingRenderer.DrawPerson(colorImage, personId, userRectangle, centerMass, centerMassWorld);
  DrawPersonSkeleton(colorImage, user);
  DrawPersonGestures(colorImage, user);
  DrawPersonLandmarks(colorImage, user);
  DrawFace(colorImage, user);
  DrawPersonSummaryReport(colorImage, user);
}
void PersonTrackingSample::DrawPersonSkeleton(cv::Mat& colorImage, realsense_ros_person::User& user)
{
  
  if (!mEnableSkeleton) return;
  std::vector<cv::Point> points;
  
  Skeleton s[6];
  for(int i=0; i<6;i++){
	s[i].x = 0;
	s[i].y = 0;
	s[i].ti = 0;	
}
   framec = framec + 1;
  for (realsense_ros_person::SkeletonJoint joint : user.skeletonJoints)
  { 
    //ROS_INFO_STREAM("joint confi <<joint.confidence);
    //ROS_INFO_STREAM("joint thresh <<JOINT_CONFIDENCE_THR);
  
   if (joint.confidence > JOINT_CONFIDENCE_THR)
    {
      //ROS_INFO_STREAM("xffgd");
     // current date/time based on current system
     time_t now = time(0);
     
   // convert now to string form
     char* dt = ctime(&now);
        gettimeofday(&tp, NULL);
	float ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	
       // ROS_INFO_STREAM("time in milli "<<ms);
      
      std::string x = std::to_string(joint.location.x);
      std::string y = std::to_string(joint.location.y);
      //ROS_INFO_STREAM("x co-ord:"<<x + " y co-ord: " <<y);
      //ROS_INFO_STREAM("The joint is"<<joint.type);
      //ROS_INFO_STREAM("frame number is"<<framec);
      //ROS_INFO_STREAM("The local date and time is: " << dt);
      int cx = stoi(x);
      int cy = stoi(y);
      float eud1, eud2, eud3, eud4, eud5, eud6;
      float time_diff1, time_diff2, time_diff3, time_diff4, time_diff5, time_diff6;
      float vel1, vel2, vel3, vel4, vel5, vel6;
      if(joint.type == 6){
	eud1 = sqrt((cx-s[0].x)*(cx-s[0].x) + (cy-s[0].y)*(cy-s[0].y));
	s[0].x = cx;
	s[0].y = cy;
	time_diff1 = ms - s[0].ti;
	s[0].ti = ms;	
	vel1 = eud1/time_diff1;
        vel1 = vel1 * pow(10,10);  
       if(vel1 >= 1)
        {ROS_INFO_STREAM("!!!!ABNORMAL ACTIVITY DETECTED!!!!!");
          count++;
          if(count == 2){
              count=0;
          ROS_INFO_STREAM(" count:"<<count);
          system("rosrun sound_play say.py \"abnormal activity\" &");
          myfile.open("/home/shruti/catkin_ws/skeleton.txt");
          if (myfile.is_open()){
          myfile << "abnormal";
          myfile.close();}
          else{
          ROS_INFO_STREAM(" ");}
          }
        }
         
	 ROS_INFO_STREAM("The velocity of left hand is"<<vel1<<" count:"<<count);
	}
       
      if(joint.type == 7){
	eud2 = sqrt((cx-s[1].x)*(cx-s[1].x) + (cy-s[1].y)*(cy-s[1].y));	
	s[1].x = cx;
	s[1].y = cy; 
	time_diff2 = ms - s[1].ti;
	s[1].ti = ms;
	vel2 = eud2/time_diff2;
        vel2 = vel2 * pow(10,10); 
        if(vel1 >= 2.7)
        {ROS_INFO_STREAM("!!!!ABNORMAL ACTIVITY DETECTED!!!!!");
          count++;
          if(count == 2){
               count=0;
          ROS_INFO_STREAM(" count:"<<count);
          system("rosrun sound_play say.py \"abnormal activity\" &");
          myfile.open("/home/shruti/catkin_ws/skeleton.txt");
          if (myfile.is_open()){
          myfile << "abnormal";
          myfile.close();}
          else{
          ROS_INFO_STREAM(" ");}
          }
          ROS_INFO_STREAM("The velocity of left hand is"<<vel2<<" count:"<<count);
        }
        
        }
      if(joint.type == 10){
	eud3 = sqrt((cx-s[2].x)*(cx-s[2].x) + (cy-s[2].y)*(cy-s[2].y));
	//ROS_INFO_STREAM("distance : "<<eud);
	s[2].x = cx;
	s[2].y = cy; 
	time_diff3 = ms - s[2].ti;
	s[2].ti = ms;
	vel3 = eud3/time_diff3;
	//ROS_INFO_STREAM("The velocity is"<<vel3);
	}
      if(joint.type == 16){
	eud4 = sqrt((cx-s[3].x)*(cx-s[3].x) + (cy-s[3].y)*(cy-s[3].y));	
	s[3].x = cx;
	s[3].y = cy; 
	time_diff4 = ms - s[3].ti;
	s[3].ti = ms;
	vel4 = eud4/time_diff4;
	//ROS_INFO_STREAM("The velocity is"<<vel4);
	}
      if(joint.type == 17){
	eud5 = sqrt((cx-s[4].x)*(cx-s[4].x) + (cy-s[4].y)*(cy-s[4].y));	
	s[4].x = cx;
	s[4].y = cy;
	time_diff5 = ms - s[4].ti;
	s[4].ti = ms; 
	vel5 = eud5/time_diff5;
	//ROS_INFO_STREAM("The velocity is"<<vel5);
	}
      if(joint.type == 19){
	eud6 = sqrt((cx-s[5].x)*(cx-s[5].x) + (cy-s[5].y)*(cy-s[5].y));	
	s[5].x = cx;
	s[5].y = cy;
	time_diff6 = ms - s[5].ti;
	s[5].ti = ms; 
	vel6 = eud6/time_diff6;
	//ROS_INFO_STREAM("The velocity is"<<vel6);
	}
	
      cv::Point location(joint.location.x, joint.location.y);
      points.push_back(location);
    }
   
  }
  m_trackingRenderer.DrawSkeleton(colorImage, points);
    

}

void PersonTrackingSample::DrawPersonLandmarks(cv::Mat& colorImage, realsense_ros_person::User& user)
{
  if (!mEnableLandmarks) return;
  if (user.landmarksInfo.confidence <  LANDMARKS_CONFIDENCE_THR) return;

  std::vector<cv::Point> points;
  for (realsense_ros_person::Landmark landmark : user.landmarksInfo.landmarks)
  {
    cv::Point location(landmark.location.x, landmark.location.y);
    points.push_back(location);
  }
  m_trackingRenderer.DrawLandmarks(colorImage, points);
}

void PersonTrackingSample::DrawFace(cv::Mat& colorImage, realsense_ros_person::User& user)
{
  if (!mEnableHeadBoundingBox) return;
  //ROS_INFO_STREAM("draw face confi %d"<<user.headBoundingBox.confidence);
  if (user.headBoundingBox.confidence > HEAD_BOUNDING_BOX_THR)
  {
    cv::Point pt1(user.headBoundingBox.rectCorners[0].x, user.headBoundingBox.rectCorners[0].y);
    cv::Point pt2(user.headBoundingBox.rectCorners[1].x, user.headBoundingBox.rectCorners[1].y);
    cv::Rect headBoundingBoxRect(pt1, pt2);
    m_trackingRenderer.DrawFace(colorImage, headBoundingBoxRect);
  }
}


void PersonTrackingSample::DrawPersonGestures(cv::Mat& colorImage, realsense_ros_person::User& user)
{
  if (!mEnablePointingGesture) return;

  if (user.gestures.pointing.confidence > 0)
  {
    realsense_ros_person::Pointing pointing = user.gestures.pointing;
    cv::Point origin(pointing.originColor.x, pointing.originColor.y);
    cv::Point2f direction(pointing.orientationColor.x, pointing.orientationColor.y);

    m_trackingRenderer.DrawPointing(colorImage, origin, direction);
  }
}

void PersonTrackingSample::PersonSelectedHandler(PersonData& data, TrackingRenderer::SelectType type)
{
  std::lock_guard<std::mutex> guard(mMutex);

  if (type == TrackingRenderer::SelectType::RECOGNITION)
  {
    realsense_ros_person::Recognition request;
    request.request.personId = data.Id;
    mRecognitionRequestClient.call(request);
    data.rid = request.response.recognitionId;
    std::string res=RecognitionResultToString(request.response.status);
    std::string s1="RECOGNITION_FAILED_FACE_NOT_DETECTED";
    std::string s2="RECOGNITION_FAILED_FACE_NOT_CLEAR";
    std::string s3="RECOGNITION_FAILED_FACE_AMBIGUITY";
    if(res.compare(s1) == 0 || res.compare(s2) == 0 || res.compare(s3) == 0){
     ROS_ERROR_STREAM("!!!DO NOT COVER FACE!!!");
     //sound_play::SoundClient sc;

     //sc.playWave("/home/shruti/Mask.wav");
     system("rosrun sound_play say.py \"Do not cover your face\" &");
     }
     else{
      ROS_INFO_STREAM("Recognition Status = " + RecognitionResultToString(request.response.status));
     }
    
  }
  else if (type == TrackingRenderer::SelectType::REGISTRATION)
  { 
    realsense_ros_person::RecognitionRegister request;
    if(data.Id != 0)
    {
       ROS_ERROR_STREAM("2 people detected!! Only one allowed at a time!!");
       system("rosrun sound_play say.py \"2 people detected Only one allowed at a time\" &");
       return;
       
    }
    else{
    request.request.personId = data.Id;
    mRegisterRequestClient.call(request);
    data.rid = request.response.recognitionId;
    
   ROS_INFO_STREAM("Registration Status = " + RegistrationResultToString(request.response.status));
   
    }

  }
  else if (type == TrackingRenderer::SelectType::TRACKING)
  {
    auto personState = ros::topic::waitForMessage<realsense_ros_person::PersonModuleState>(PERSON_MODULE_STATE_TOPIC, ros::Duration(5));
    if (personState == nullptr)
    {
      ROS_ERROR_STREAM("Failed to get person tracking state");
      return;
    }
   if (personState->trackingState == realsense_ros_person::PersonModuleState::TRACKING_STATE_DETECTING)
    {
      realsense_ros_person::StartTracking request;
      request.request.personId = data.Id;
      mStartTrackingRequestClient.call(request);
      std::string res = request.response.status ? " SUCCEEDED" : " FAILED";
     ROS_INFO_STREAM("Start tracking of user ID " + std::to_string(data.Id) + res);
    }
    else
    {
      realsense_ros_person::StopTracking request;
      request.request.personId = data.Id;
      mStopTrackingRequestClient.call(request);
      std::string res = request.response.status ? " SUCCEEDED" : " FAILED";
      ROS_INFO_STREAM("Stop tracking of user ID " + std::to_string(data.Id) + res);
    }
  }
}

//Function For Handling glbal events (that are not specific for a user)
void PersonTrackingSample::GlobalHandler(TrackingRenderer::SelectType type)
{
//    std::lock_guard<std::mutex> guard(mMutex);
}


void PersonTrackingSample::DrawPersonSummaryReport(cv::Mat image, realsense_ros_person::User &user)
{
  std::stringstream summaryText;// summary text at at top left corner of image (center of mass, orientation etc.)

  //add center of mass (world coordinates)
  summaryText << user.userInfo.Id << ": " <<
              std::fixed << std::setprecision(3) <<
              "(" << user.centerOfMassWorld.x << "," << user.centerOfMassWorld.y << "," << user.centerOfMassWorld.z << ")";

  if (user.headPose.confidence > 0)
  {
    //add head pose
    summaryText << std::setprecision(1) << std::fixed << " head orientation " << "(" <<
                user.headPose.angles.pitch << ", " <<
                user.headPose.angles.roll << ", " <<
                user.headPose.angles.yaw << ")";
  }

  //add wave gesture
  int32_t waveGesture = user.gestures.wave.type;
  if (waveGesture != (int32_t)realsense_ros_person::Wave::WAVE_NOT_DETECTED)
  {
    summaryText << " wave gesture: " << WaveGestureToString(waveGesture).c_str() << "\n";
  }

  m_trackingRenderer.DrawLineAtSummaryReport(image, summaryText.str());
}

std::string RegistrationResultToString(int status)
{
  switch (status)
  {
  case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_SUCCESSFULL:
    return "REGISTRATION_SUCCESSFULL";
  case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED:
    return "REGISTRATION_FAILED";
  case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_ALREADY_REGISTERED:
    return "REGISTRATION_FAILED_ALREADY_REGISTERED";
  case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_FACE_NOT_DETECTED:
    return "REGISTRATION_FAILED_FACE_NOT_DETECTED";
  case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_FACE_NOT_CLEAR:
    return "REGISTRATION_FAILED_FACE_NOT_CLEAR";
  case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_PERSON_TO_FAR:
    return "REGISTRATION_FAILED_PERSON_TO_FAR";
  case realsense_ros_person::RecognitionRegisterResponse::REGISTRATION_FAILED_PERSON_TO_CLOSE:
    return "REGISTRATION_FAILED_PERSON_TO_CLOSE";
  default:
    return "REGISTRATION_UNKNOWN_ERROR";
  }
}


std::string RecognitionResultToString(int status)
{
  switch (status)
  {
  case realsense_ros_person::RecognitionResponse::RECOGNITION_PASSED_PERSON_RECOGNIZED:
    return "RECOGNITION_PASSED_PERSON_RECOGNIZED";
  case realsense_ros_person::RecognitionResponse::RECOGNITION_PASSED_PERSON_NOT_RECOGNIZED:
    return "RECOGNITION_PASSED_PERSON_NOT_RECOGNIZED";
  case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED:
    return "RECOGNITION_FAILED";
  case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_FACE_NOT_DETECTED:
    return "RECOGNITION_FAILED_FACE_NOT_DETECTED";
  case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_FACE_NOT_CLEAR:
    return "RECOGNITION_FAILED_FACE_NOT_CLEAR";
  case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_PERSON_TOO_FAR:
    return "RECOGNITION_FAILED_PERSON_TOO_FAR";
  case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_PERSON_TOO_CLOSE:
    return "RECOGNITION_FAILED_PERSON_TOO_CLOSE";
  case realsense_ros_person::RecognitionResponse::RECOGNITION_FAILED_FACE_AMBIGUITY:
    return "RECOGNITION_FAILED_FACE_AMBIGUITY";
  default:
    return "RECOGNITION_UNKNOWN_ERROR";
  }
}

std::string WaveGestureToString(int32_t waveGestureRos)
{
  switch (waveGestureRos)
  {
  case realsense_ros_person::Wave::WAVE_NOT_DETECTED:
    return "Wave not detected";
  case realsense_ros_person::Wave::WAVE_LEFT_LA:
    return "Wave left left area";
  case realsense_ros_person::Wave::WAVE_RIGHT_LA:
    return "Wave right left area";
  case realsense_ros_person::Wave::WAVE_LEFT_RA:
    return "Wave left right area";
  case  realsense_ros_person::Wave::WAVE_RIGHT_RA:
    return "Wave right right area";
  default:
    throw std::runtime_error("unsupported wave gesture value");
  }
}
