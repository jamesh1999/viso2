#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
//#include <pcl_ros/point_cloud.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <viso_stereo.h>

#include <viso2_ros/msg/viso_info.hpp>

#include "stereo_processor.h"
#include "odometer_base.h"
#include "odometry_params.h"

// to remove after debugging
#include <opencv2/highgui/highgui.hpp>

namespace viso2_ros
{

// some arbitrary values (0.1m^2 linear cov. 10deg^2. angular cov.)
static const std::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.17, 0, 0,
    0, 0, 0, 0, 0.17, 0,
    0, 0, 0, 0, 0, 0.17 } };
static const std::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.002, 0, 0, 0, 0, 0,
    0, 0.002, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.09, 0, 0,
    0, 0, 0, 0, 0.09, 0,
    0, 0, 0, 0, 0, 0.09 } };
static const std::array<double, 36> BAD_COVARIANCE =
{ { 9999, 0, 0, 0, 0, 0,
    0, 9999, 0, 0, 0, 0,
    0, 0, 9999, 0, 0, 0,
    0, 0, 0, 9999, 0, 0,
    0, 0, 0, 0, 9999, 0,
    0, 0, 0, 0, 0, 9999 } };


class StereoOdometer : public StereoProcessor, public OdometerBase
{

private:

  std::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryStereo::parameters visual_odometer_params_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<viso2_ros::msg::VisoInfo>::SharedPtr info_pub_;

  bool got_lost_;

  // change reference frame method. 0, 1 or 2. 0 means allways change. 1 and 2 explained below
  int ref_frame_change_method_;
  bool change_reference_frame_;
  double ref_frame_motion_threshold_; // method 1. Change the reference frame if last motion is small
  int ref_frame_inlier_threshold_; // method 2. Change the reference frame if the number of inliers is low
  Matrix reference_motion_;

public:

  StereoOdometer(const rclcpp::Node::SharedPtr node) :
    StereoProcessor(transport, node), 
    OdometerBase(node),
    got_lost_(false), 
    change_reference_frame_(false)
  {

    node_ = node;
    // Read local parameters
    odometry_params::loadParams(node_, visual_odometer_params_);

    auto transport = node_->declare_parameter("transport", "raw");

    auto ref_frame_change_method_ = node_->declare_parameter("ref_frame_change_method", 0);
    auto ref_frame_motion_threshold_ = node_->declare_parameter("ref_frame_motion_threshold", 5.0);
    auto ref_frame_inlier_threshold_ = node_->declare_parameter("ref_frame_inlier_threshold", 150);

    point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 1);
    info_pub_ = node_->create_publisher<viso2_ros::msg::VisoInfo>("info", 1);

    reference_motion_ = Matrix::eye(4);
  }

protected:

  void initOdometer(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr l_info_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr r_info_msg)
  {
    int queue_size;
    bool approximate_sync;
    queue_size = this->declare_parameter("queue_size", 5);
    approximate_sync = this->declare_parameter("approximate_sync", false);

    // read calibration info from camera info message
    // to fill remaining parameters
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(*l_info_msg, *r_info_msg);
    visual_odometer_params_.base      = model.baseline();
    visual_odometer_params_.calib.cu  = model.left().cx();
    visual_odometer_params_.calib.cv  = model.left().cy();
    visual_odometer_params_.calib.f   = model.left().fx();

    visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
    if (l_info_msg->header.frame_id != "") {
      setSensorFrameId(l_info_msg->header.frame_id);
    }
    RCLCPP_INFO(this->get_logger(), "Initialized libviso2 stereo odometry with the following parameters: %s \n  queue_size = %s \n approximate_sync = %s \n ref_frame_change_method = %s \n ref_frame_motion_threshold = %s \n ref_frame_inlier_threshold = %s",
                    visual_odometer_params_ ,
                    queue_size, 
                    approximate_sync, 
                    ref_frame_change_method_, 
                    ref_frame_motion_threshold_,  
                    ref_frame_inlier_threshold_);
  }

  void imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr l_image_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr r_image_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr l_info_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr r_info_msg)
  {
    auto start_time = rclcpp::Clock().now();
    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_)
    {
      first_run = true;
      initOdometer(l_info_msg, r_info_msg);
    }

    // convert images if necessary
    uint8_t *l_image_data, *r_image_data;
    uint32_t l_step, r_step;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    l_image_data = l_cv_ptr->image.data;
    l_step = l_cv_ptr->image.step[0];
    r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    r_image_data = r_cv_ptr->image.data;
    r_step = r_cv_ptr->image.step[0];

    //ROS_ASSERT(l_step == r_step);
    //ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    //ROS_ASSERT(l_image_msg->height == r_image_msg->height);

    int32_t dims[] = {(int32_t)l_image_msg->width, (int32_t)l_image_msg->height, (int32_t)l_step};
    // on first run or when odometer got lost, only feed the odometer with
    // images without retrieving data
    if (first_run || got_lost_)
    {
      visual_odometer_->process(l_image_data, r_image_data, dims);
      got_lost_ = false;
      // on first run publish zero once
      if (first_run)
      {
        tf2::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);
      }
    }
    else
    {
      bool success = visual_odometer_->process(
          l_image_data, r_image_data, dims, change_reference_frame_);
      if (success)
      {
        Matrix motion = Matrix::inv(visual_odometer_->getMotion());
        RCLCPP_DEBUG(this->get_logger(), "Found %i matches with %i inliers.",
                  visual_odometer_->getNumberOfMatches(),
                  visual_odometer_->getNumberOfInliers());
        //RCLCPP_DEBUG(this->get_logger(), "libviso2 returned the following motion: %s \n", motion);
        Matrix camera_motion;
        // if image was replaced due to small motion we have to subtract the
        // last motion to get the increment
        if (change_reference_frame_)
        {
          camera_motion = Matrix::inv(reference_motion_) * motion;
        }
        else
        {
          // image was not replaced, report full motion from odometer
          camera_motion = motion;
        }
        reference_motion_ = motion; // store last motion as reference

        tf2::Matrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        tf2::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf2::Transform delta_transform(rot_mat, t);

        setPoseCovariance(STANDARD_POSE_COVARIANCE);
        setTwistCovariance(STANDARD_TWIST_COVARIANCE);

        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        if (this->count_subscribers(point_cloud_pub_.getTopic()) > 0)
        {
          computeAndPublishPointCloud(l_info_msg, l_image_msg, r_info_msg,
                                      visual_odometer_->getMatches(),
                                      visual_odometer_->getInlierIndices());
        }
      }
      else
      {
        setPoseCovariance(BAD_COVARIANCE);
        setTwistCovariance(BAD_COVARIANCE);
        tf2::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        RCLCPP_DEBUG(this->get_logger(), "Call to VisualOdometryStereo::process() failed.");
        RCLCPP_WARN(this->get_logger(), "10.0 Visual Odometer got lost!");
        got_lost_ = true;
      }

      if(success)
      {

        // Proceed depending on the reference frame change method
        switch ( ref_frame_change_method_ )
        {
          case 1:
          {
            // calculate current feature flow
            double feature_flow = computeFeatureFlow(visual_odometer_->getMatches());
            change_reference_frame_ = (feature_flow < ref_frame_motion_threshold_);
            RCLCPP_DEBUG(this->get_logger(), "Feature flow is %s ,marking last motion as %s" , feature_flow, (change_reference_frame_ ? "small." : "normal."));
            break;
          }
          case 2:
          {
            change_reference_frame_ = (visual_odometer_->getNumberOfInliers() > ref_frame_inlier_threshold_);
            break;
          }
          default:
            change_reference_frame_ = false;
        }

      }
      else
        change_reference_frame_ = false;

      if(!change_reference_frame_)
        RCLCPP_DEBUG(this->get_logger(), "Changing reference frame");

      // create and publish viso2 info msg
      viso2_ros::msg::VisoInfo info_msg;
      info_msg.header.stamp = l_image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = !change_reference_frame_;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      rclcpp::Duration time_elapsed = rclcpp::Clock().now() - start_time;
      info_msg.runtime = time_elapsed.seconds();
      info_pub_->publish(info_msg);
    }
  }

  double computeFeatureFlow(
      const std::vector<Matcher::p_match>& matches)
  {
    double total_flow = 0.0;
    for (size_t i = 0; i < matches.size(); ++i)
    {
      double x_diff = matches[i].u1c - matches[i].u1p;
      double y_diff = matches[i].v1c - matches[i].v1p;
      total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    return total_flow / matches.size();
  }

  void computeAndPublishPointCloud(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& l_info_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr l_image_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr r_info_msg,
      const std::vector<Matcher::p_match>& matches,
      const std::vector<int32_t>& inlier_indices)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
      // read calibration info from camera info message
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      PointCloud::Ptr point_cloud(new PointCloud());
      point_cloud->header.frame_id = getSensorFrameId();
      point_cloud->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
      point_cloud->width = 1;
      point_cloud->height = inlier_indices.size();
      point_cloud->points.resize(inlier_indices.size());

      for (size_t i = 0; i < inlier_indices.size(); ++i)
      {
        const Matcher::p_match& match = matches[inlier_indices[i]];
        cv::Point2d left_uv;
        left_uv.x = match.u1c;
        left_uv.y = match.v1c;
        cv::Point3d point;
        double disparity = match.u1c - match.u2c;
        model.projectDisparityTo3d(left_uv, disparity, point);
        point_cloud->points[i].x = point.x;
        point_cloud->points[i].y = point.y;
        point_cloud->points[i].z = point.z;
        cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(left_uv.y,left_uv.x);
        point_cloud->points[i].r = color[0];
        point_cloud->points[i].g = color[1];
        point_cloud->points[i].b = color[2];
      }
      RCLCPP_DEBUG(this->get_logger(), "Publishing point cloud with %zu points.", point_cloud->size());
      point_cloud_pub_->publish(*point_cloud);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }
};

} // end of namespace
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  /*if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("mono_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }*/

  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("stereo_odometer_node", options)
  auto odometer = std::make_shared<viso2_ros::StereoOdometer>(node);
  
  exec.add_node(node);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}

