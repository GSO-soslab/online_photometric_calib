// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// OPC
#include "StandardIncludes.h"
#include "RapidExposureTimeEstimator.h"
#include "Database.h"
#include <optimizer/NonlinearOptimizer.h>
#include <tracker/Tracker.h>

#include <tic_toc.h>

class OnlinePhotometricCalib
{
public:
  OnlinePhotometricCalib(ros::NodeHandle* nh, OptimizationParam &param) :
    nh_(*nh), it_(nh_), opt_params_(param), vis_exponent(1.0), 
    img_count(0), is_optimizing(false), is_updated(true),
    database(opt_params_.image_width, opt_params_.image_height),
    exposure_estimator(opt_params_.nr_images_rapid_exp, &database), 
    backend_optimizer(opt_params_.keyframe_spacing, &database, opt_params_.safe_zone_size, 
                      opt_params_.min_keyframes_valid, opt_params_.tracker_patch_size), 
    tracker(opt_params_.tracker_patch_size,opt_params_.nr_active_features,
            opt_params_.nr_pyramid_levels, &database)
  {
    
    sub = it_.subscribe("image", 1, &OnlinePhotometricCalib::imageCallback, this);

    tracking_img_pub = it_.advertise("/tracking_img", 10);
    correct_img_pub = it_.advertise("/correct_img", 10);
  }

  ~OnlinePhotometricCalib()
  {
      // destroy something;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void optimization();

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber sub;
  image_transport::Publisher tracking_img_pub;
  image_transport::Publisher correct_img_pub;

  OptimizationParam opt_params_;

  Database database;

  RapidExposureTimeEstimator exposure_estimator;

  NonlinearOptimizer backend_optimizer;

  Tracker tracker;

  double vis_exponent;

  int img_count;

  std::atomic<bool> is_optimizing;
  std::atomic<bool> is_updated;

};

void OnlinePhotometricCalib::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  TicToc t;

  // get image
  cv_bridge::CvImagePtr cv_ptr0;
  try
  {
    cv_ptr0 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);//BGR8
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'MONO8'.", msg->encoding.c_str());
  }

  // image enhacement
  cv::Mat img;
  img = cv_ptr0->image.clone();
  // cv::equalizeHist(cv_ptr0->image, img);


/**** OPC ****/
  // get groundtruth exposure time
  double gt_exp_time = -1.0;

  // remove frames no exposure time could be optimized 
  if(img_count == opt_params_.nr_images_rapid_exp*2 + opt_params_.safe_zone_size)
  {
    for(int ii = 0; ii < opt_params_.nr_images_rapid_exp; ii++)
      database.removeLastFrame();
  }

  // If the database is large enough, start removing old frames
  if(img_count > opt_params_.nr_active_frames)
    database.removeLastFrame();

  // track image
  cv::resize(img, img, cv::Size(opt_params_.image_width, opt_params_.image_height));
  tracker.trackNewFrame(img, gt_exp_time);

  // Rapid exposure time estimation (+ time the result)
  double exposure_time = exposure_estimator.estimateExposureTime();
  database.m_tracked_frames.at(database.m_tracked_frames.size()-1).m_exp_time = exposure_time;
  database.visualizeRapidExposureTimeEstimates(vis_exponent);

  // Remove the exposure time from the radiance estimates
  std::vector<Feature*>* features = &database.m_tracked_frames.at(database.m_tracked_frames.size()-1).m_features;
  for(int k = 0;k < features->size();k++)
  {
      for(int r = 0;r < features->at(k)->m_radiance_estimates.size();r++)
      {
          features->at(k)->m_radiance_estimates.at(r) /= exposure_time;
      }
  }

  // Visualize tracking
  if(img_count%opt_params_.visualize_cnt == 0) {
      cv::Mat tracking_img = tracker.get_visualized_img();
      cv::Mat correct_img = database.getCorrectImg();

      sensor_msgs::ImagePtr tracking_msg = cv_bridge::CvImage(msg->header, "bgr8", tracking_img).toImageMsg();
      tracking_img_pub.publish(tracking_msg);
      sensor_msgs::ImagePtr correct_msg = cv_bridge::CvImage(msg->header, "mono8", correct_img).toImageMsg();
      correct_img_pub.publish(correct_msg);
  }

  // get optimization result if it's finished, 'is_update' used in case first time
  if (!is_optimizing && !is_updated) {

    database.m_vignette_estimate.setVignetteParameters(backend_optimizer.m_vignette_estimate);
    database.m_response_estimate.setGrossbergParameterVector(backend_optimizer.m_response_estimate);
    database.m_response_estimate.setInverseResponseVector(backend_optimizer.m_raw_inverse_response);
    
    vis_exponent = backend_optimizer.visualizeOptimizationResult(backend_optimizer.m_raw_inverse_response);

    is_updated = true;
  }

  img_count++;
  std::cout<<"i: "<<img_count<<std::endl;

  // ROS_INFO("time: %.2fms", t.toc());

  //TODO: when last image is over, the previous optimization maybe still running, 
  //      need to update those, like in the original code?
}

void OnlinePhotometricCalib::optimization()
{
  while(1)
  {
    // start a new optimization after optimization result is updated in main thread
    if (is_updated) {

      // Try to fetch a new optimization block
      bool succeeded = backend_optimizer.extractOptimizationBlock();

      if (succeeded) {
        ROS_INFO("OPC: START OPTIMIZATION !");

        // multi-thread flags
        is_optimizing= true;
        is_updated = false;
        // Get Response and Vignette from database
        backend_optimizer.fetchResponseVignetteFromDatabase();
        // Perform optimization
        backend_optimizer.evfOptimization(false); // crush from here
        backend_optimizer.evfOptimization(false);
        backend_optimizer.evfOptimization(false);
        // Smooth optimization data
        backend_optimizer.smoothResponse();
        // Initialize the inverse response vector with the current inverse response estimate
        // (in order to write it to the database later + visualization)
        // better to do this here since currently the inversion is done rather inefficiently and not to slow down tracking
        backend_optimizer.getInverseResponseRaw(backend_optimizer.m_raw_inverse_response);
        // Finsih Optimization
        is_optimizing= false;
        
        ROS_INFO("OPC: END OPTIMIZATION !");
      }
    }

    // sleep for some duration
    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

void loadParameters(ros::NodeHandle nh, OptimizationParam &params)
{
  // get params from config.ymal
  nh.param<int>("basic/image_width", params.image_width, 640);
  nh.param<int>("basic/image_height", params.image_height, 480);
  nh.param<int>("basic/visualize_cnt", params.visualize_cnt, 1);
  nh.param<int>("basic/tracker_patch_size", params.tracker_patch_size, 3);
  nh.param<int>("basic/nr_pyramid_levels", params.nr_pyramid_levels, 2);
  nh.param<int>("basic/nr_active_features", params.nr_active_features, 200);
  nh.param<int>("basic/nr_images_rapid_exp", params.nr_images_rapid_exp, 15);
  nh.param<int>("basic/safe_zone_size", params.safe_zone_size, 20);
  nh.param<int>("basic/nr_active_frames", params.nr_active_frames, 200);
  nh.param<int>("basic/keyframe_spacing", params.keyframe_spacing, 15);
  nh.param<int>("basic/min_keyframes_valid", params.min_keyframes_valid, 3);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OPC_Node");
  ros::NodeHandle nh;
  // get parameters
  OptimizationParam opt_params;
  loadParameters(nh, opt_params);
  // start OPC node
  OnlinePhotometricCalib  node(&nh, opt_params);

  std::thread optimization_thread{&OnlinePhotometricCalib::optimization, &node};

  ros::spin();

  // optimization_thread.join();

  return 0;
}