/// ROS 2 wrapper for OpenFace for one face, publishing output under separate topics

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS 2 interfaces
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <openface_msgs/msg/pixels_stamped.hpp>
#include <openface_msgs/msg/points_stamped.hpp>
#include <openface_msgs/msg/action_units_stamped.hpp>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

// OpenFace
#include <OpenFace/FaceAnalyser.h>
#include <OpenFace/LandmarkCoreIncludes.h>
#include <OpenFace/GazeEstimation.h>
#include <OpenFace/Visualizer.h>
#include <OpenFace/VisualizationUtils.h>

//////////////////
/// NAMESPACES ///
//////////////////

using namespace std::placeholders;

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "openface";
/// Number of failures before landmark detection is reinitialised
const int LANDMARK_DETECTION_MAX_FAILURES_BEFORE_REINIT = 2;
/// Name of the TF frame for head
const std::string TF_FRAME_HEAD = "head";
/// Name of the TF frame for left eye/pupil
const std::string TF_FRAME_EYE_LEFT = "eye_left";
/// Name of the TF frame for right eye/pupil
const std::string TF_FRAME_EYE_RIGHT = "eye_right";

//////////////////
/// NODE CLASS ///
//////////////////

/// Class representation of this node
class Ros2OpenFaceSeparate : public rclcpp::Node
{
public:
  /// Constructor
  Ros2OpenFaceSeparate();

private:
  /// Subscriber to the camera, using image_transport
  image_transport::CameraSubscriber sub_camera_;
  /// Publisher of the OpenFace landmarks
  rclcpp::Publisher<openface_msgs::msg::PixelsStamped>::SharedPtr pub_landmarks_;
  /// Publisher of the OpenFace visible landmarks
  rclcpp::Publisher<openface_msgs::msg::PixelsStamped>::SharedPtr pub_landmarks_visible_;
  /// Publisher of the OpenFace 3D landmarks
  rclcpp::Publisher<openface_msgs::msg::PointsStamped>::SharedPtr pub_landmarks_3d_;
  /// Publisher of the OpenFace head pose
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_head_pose_;
  /// Publisher of the OpenFace eye landmarks
  rclcpp::Publisher<openface_msgs::msg::PixelsStamped>::SharedPtr pub_eye_landmarks_;
  /// Publisher of the OpenFace visible eye landmarks
  rclcpp::Publisher<openface_msgs::msg::PixelsStamped>::SharedPtr pub_eye_landmarks_visible_;
  /// Publisher of the OpenFace 3D eye landmarks
  rclcpp::Publisher<openface_msgs::msg::PointsStamped>::SharedPtr pub_eye_landmarks_3d_;
  /// Publisher of the OpenFace left gaze
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gaze_left_;
  /// Publisher of the OpenFace right gaze
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gaze_right_;
  /// Publisher of the OpenFace right gaze
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_gaze_compound_;
  /// Publisher of the OpenFace Action Units
  rclcpp::Publisher<openface_msgs::msg::ActionUnitsStamped>::SharedPtr pub_action_units_;
  /// Broadcaster of the TF frames acquired by OpenFace
  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  /// Parameters for the OpenFace face model
  LandmarkDetector::FaceModelParameters det_parameters_;
  /// OpenFace CLNF face landmark detector
  LandmarkDetector::CLNF face_landmark_detector_;
  /// OpenFace face analyser
  FaceAnalysis::FaceAnalyser face_analyser_;
  /// OpenFace visualiser
  Utilities::Visualizer visualiser_;
  /// OpenFace FPS tracker
  Utilities::FpsTracker fps_tracker_;

  /// Function that initialises OpenFace
  void initialise_openface();
  /// Callback called each time a new frame is obtained
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam);
};

Ros2OpenFaceSeparate::Ros2OpenFaceSeparate() : Node(NODE_NAME),
                                               tf2_broadcaster_(this),
                                               face_analyser_([]() -> FaceAnalysis::FaceAnalyser { FaceAnalysis::FaceAnalyserParameters face_analysis_params; 
                               face_analysis_params.OptimizeForImages();
                               return  FaceAnalysis::FaceAnalyser(face_analysis_params); }()),
                                               visualiser_(true, true, true, true)
{
  // Create a subscriber to camera that provides frames for processing
  sub_camera_ = image_transport::create_camera_subscription(this, "camera/image_raw", image_transport::CameraSubscriber::Callback(std::bind(&Ros2OpenFaceSeparate::image_callback, this, _1, _2)), "raw", rmw_qos_profile_sensor_data);

  // Declaration of the available parameters
  this->declare_parameter<std::vector<bool>>("enable_face_features", std::vector<bool>(3, true)); // 2D landmarks, 2D visible landmarks, 3D landmarks
  this->declare_parameter<bool>("enable_head_pose", true);

  this->declare_parameter<std::vector<bool>>("enable_eye_features", std::vector<bool>(3, true)); // 2D landmarks, 2D visible landmarks, 3D landmarks
  this->declare_parameter<bool>("enable_gaze", true);

  this->declare_parameter<bool>("enable_action_units", true);
  this->declare_parameter<bool>("enable_aligned_face", true); // Only for visualisation
  this->declare_parameter<bool>("enable_hog", true);          // Only for visualisation

  this->declare_parameter<bool>("broadcast_tf", true);
  this->declare_parameter<bool>("visualise", true);
  this->declare_parameter<bool>("track_fps", true);

  // TODO: Implement run-time setting of parameters

  // Create publishers for information obtained from OpenFace
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

  auto enable_face_features = this->get_parameter("enable_face_features").get_value<std::vector<bool>>();
  if (enable_face_features[0])
  {
    pub_landmarks_ = this->create_publisher<openface_msgs::msg::PixelsStamped>(NODE_NAME + "/landmarks", qos);
  }
  if (enable_face_features[1])
  {
    pub_landmarks_visible_ = this->create_publisher<openface_msgs::msg::PixelsStamped>(NODE_NAME + "/landmarks_visible", qos);
  }
  if (enable_face_features[2])
  {
    pub_landmarks_3d_ = this->create_publisher<openface_msgs::msg::PointsStamped>(NODE_NAME + "/landmarks_3d", qos);
  }

  if (this->get_parameter("enable_head_pose").get_value<bool>())
  {
    pub_head_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(NODE_NAME + "/head_pose", qos);
  }

  auto enable_eye_features = this->get_parameter("enable_eye_features").get_value<std::vector<bool>>();
  if (enable_eye_features[0])
  {
    pub_eye_landmarks_ = this->create_publisher<openface_msgs::msg::PixelsStamped>(NODE_NAME + "/eye_landmarks", qos);
  }
  if (enable_eye_features[1])
  {
    pub_eye_landmarks_visible_ = this->create_publisher<openface_msgs::msg::PixelsStamped>(NODE_NAME + "/eye_landmarks_visible", qos);
  }
  if (enable_eye_features[2])
  {
    pub_eye_landmarks_3d_ = this->create_publisher<openface_msgs::msg::PointsStamped>(NODE_NAME + "/eye_landmarks_3d", qos);
  }

  if (this->get_parameter("enable_gaze").get_value<bool>())
  {
    pub_gaze_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(NODE_NAME + "/gaze_left", qos);
    pub_gaze_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(NODE_NAME + "/gaze_right", qos);
    pub_gaze_compound_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(NODE_NAME + "/gaze_compound", qos);
  }

  if (this->get_parameter("enable_action_units").get_value<bool>())
  {
    pub_action_units_ = this->create_publisher<openface_msgs::msg::ActionUnitsStamped>(NODE_NAME + "/action_units", qos);
  }

  // Initialise OpenFace specifics
  initialise_openface();

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void Ros2OpenFaceSeparate::initialise_openface()
{
  face_landmark_detector_ = LandmarkDetector::CLNF(det_parameters_.model_location);
  if (!face_landmark_detector_.loaded_successfully)
  {
    RCLCPP_FATAL(this->get_logger(), "Cannot load landmark detector model");
    exit(EXIT_FAILURE);
  }

  // Attemp to load face detectors with priority MTCNN > HAAR > HOG_SVM
  face_landmark_detector_.face_detector_MTCNN.Read(det_parameters_.mtcnn_face_detector_location);
  face_landmark_detector_.mtcnn_face_detector_location = det_parameters_.mtcnn_face_detector_location;
  face_landmark_detector_.face_detector_HAAR.load(det_parameters_.haar_face_detector_location);
  face_landmark_detector_.haar_face_detector_location = det_parameters_.haar_face_detector_location;
  if (!face_landmark_detector_.face_detector_MTCNN.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Selecting MTCNN detector");
    det_parameters_.curr_face_detector = LandmarkDetector::FaceModelParameters::MTCNN_DETECTOR;
  }
  else if (!face_landmark_detector_.face_detector_HAAR.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot find MTCNN detector");
    RCLCPP_INFO(this->get_logger(), "Selecting HAAR detector");
    det_parameters_.curr_face_detector = LandmarkDetector::FaceModelParameters::HAAR_DETECTOR;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Cannot find MTCNN or HAAR detector");
    RCLCPP_INFO(this->get_logger(), "Selecting HOG_SVM detector");
    det_parameters_.curr_face_detector = LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR;
  }

  if (!face_landmark_detector_.eye_model && this->get_parameter("enable_gaze").get_value<bool>())
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot find any eye model");
  }

  if (face_analyser_.GetAUClassNames().empty() && this->get_parameter("enable_action_units").get_value<bool>())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot find any Action Unit model");
  }
}

void Ros2OpenFaceSeparate::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam)
{
  RCLCPP_DEBUG(this->get_logger(), "Received frame for processing");

  // Use cv_bridge to convert sensor_msgs::msg::Image to cv::Mat
  cv_bridge::CvImagePtr cv_img_bgr8, cv_img_gray8;
  try
  {
    cv_img_bgr8 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv_img_gray8 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  catch (const cv_bridge::Exception exception)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid frame. Exception from cv_bridge: %s", exception.what());
    return;
  }

  if (this->get_parameter("track_fps").get_value<bool>())
  {
    fps_tracker_.AddFrame();
  }

  // Detect landmarks of the face
  if (face_landmark_detector_.failures_in_a_row > LANDMARK_DETECTION_MAX_FAILURES_BEFORE_REINIT)
  {
    // Reinitialise landmark detector if model fails too many times in a row
    face_landmark_detector_.Reset();

    // Detect all faces within the image
    std::vector<cv::Rect_<float>> face_detections;
    int confident_face_index = 0;
    {
      std::vector<float> confidences;
      if (!face_landmark_detector_.face_detector_MTCNN.empty())
      {
        LandmarkDetector::DetectFacesMTCNN(face_detections, cv_img_bgr8->image, face_landmark_detector_.face_detector_MTCNN, confidences);
      }
      else if (!face_landmark_detector_.face_detector_HAAR.empty())
      {
        LandmarkDetector::DetectFaces(face_detections, cv_img_gray8->image, face_landmark_detector_.face_detector_HAAR);
      }
      else
      {
        LandmarkDetector::DetectFacesHOG(face_detections, cv_img_gray8->image, face_landmark_detector_.face_detector_HOG, confidences);
      }

      // Return if no face can be detected
      if (face_detections.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Cannot detect any face");
        return;
      }

      // Find the face with largest confidence (Not applicable for HAAR, in which case the first detection is considered)
      if (!confidences.empty())
      {
        confident_face_index = std::distance(confidences.begin(), std::max_element(confidences.begin(), confidences.end()));
      }
    }

    // Detect landmarks in ROI of the detected face
    LandmarkDetector::DetectLandmarksInVideo(cv_img_bgr8->image, face_detections[confident_face_index], face_landmark_detector_, det_parameters_, cv_img_gray8->image);
  }
  else
  {
    // Detect landmarks based on the current model, without reinitialising
    LandmarkDetector::DetectLandmarksInVideo(cv_img_bgr8->image, face_landmark_detector_, det_parameters_, cv_img_gray8->image);
  }

  // Continue with processing if landmark detection was successful
  if (face_landmark_detector_.detection_success)
  {
    // Extract focal length and principal point from the camera matrix
    double fx = cam->p[0];
    double fy = cam->p[5];
    double cx = cam->p[2];
    double cy = cam->p[6];
    RCLCPP_DEBUG(this->get_logger(), "Received CameraInfo: ([fx, fy], [cx, cy]) = ([%f, %f], [%f, %f])", fx, fy, cx, cy);

    // Initialise TF broadcast, if desired
    geometry_msgs::msg::TransformStamped transform;
    if (this->get_parameter("broadcast_tf").get_value<bool>())
    {
      transform.header = img->header;
    }

    // Initialise visualiser, if desired
    if (this->get_parameter("visualise").get_value<bool>())
    {
      decltype(cv_img_bgr8->image) viz_img = cv_img_bgr8->image.clone();
      visualiser_.SetImage(viz_img, fx, fy, cx, cy);
      visualiser_.SetObservationLandmarks(face_landmark_detector_.detected_landmarks, face_landmark_detector_.detection_certainty);
    }

    // Extract facial landmarks and copy to the message, if desired
    auto enable_face_features = this->get_parameter("enable_face_features").get_value<std::vector<bool>>();
    if (enable_face_features[0]) // 2D landmarks
    {
      openface_msgs::msg::PixelsStamped msg_landmarks;
      msg_landmarks.header = img->header;

      std::vector<cv::Point2f> landmarks = LandmarkDetector::CalculateAllLandmarks(face_landmark_detector_);
      for (auto landmark : landmarks)
      {
        openface_msgs::msg::Pixel landmark_px;
        landmark_px.x = landmark.x;
        landmark_px.y = landmark.y;
        msg_landmarks.pixels.push_back(landmark_px);
      }

      pub_landmarks_->publish(msg_landmarks);
    }
    if (enable_face_features[1]) // 2D visible landmarks
    {
      openface_msgs::msg::PixelsStamped msg_landmarks_visible;
      msg_landmarks_visible.header = img->header;

      std::vector<cv::Point2f> landmarks_visible = LandmarkDetector::CalculateVisibleLandmarks(face_landmark_detector_);
      for (auto landmark : landmarks_visible)
      {
        openface_msgs::msg::Pixel landmark_px;
        landmark_px.x = landmark.x;
        landmark_px.y = landmark.y;
        msg_landmarks_visible.pixels.push_back(landmark_px);
      }

      pub_landmarks_visible_->publish(msg_landmarks_visible);
    }
    if (enable_face_features[2]) // 3D landmarks
    {
      openface_msgs::msg::PointsStamped msg_landmarks_3d;
      msg_landmarks_3d.header = img->header;

      cv::Mat_<double> shape_3d = face_landmark_detector_.GetShape(fx, fy, cx, cy);
      for (auto i = 0; i < face_landmark_detector_.pdm.NumberOfPoints(); ++i)
      {
        geometry_msgs::msg::Point landmark_3d;
        landmark_3d.x = shape_3d.at<double>(i) / 1000.0;
        landmark_3d.y = shape_3d.at<double>(face_landmark_detector_.pdm.NumberOfPoints() + i) / 1000.0;
        landmark_3d.z = shape_3d.at<double>(face_landmark_detector_.pdm.NumberOfPoints() * 2 + i) / 1000.0;
        msg_landmarks_3d.points.push_back(landmark_3d);
      }

      pub_landmarks_3d_->publish(msg_landmarks_3d);
    }

    if (this->get_parameter("visualise").get_value<bool>())
    {
      visualiser_.SetObservationLandmarks(face_landmark_detector_.detected_landmarks, face_landmark_detector_.detection_certainty);
    }

    // Get head pose, if desired
    if (this->get_parameter("enable_head_pose").get_value<bool>())
    {
      geometry_msgs::msg::PoseStamped msg_head_pose;
      msg_head_pose.header = img->header;

      cv::Vec6d head_pose = LandmarkDetector::GetPose(face_landmark_detector_, fx, fy, cx, cy);
      msg_head_pose.pose.position.x = head_pose[0] / 1000.0;
      msg_head_pose.pose.position.y = head_pose[1] / 1000.0;
      msg_head_pose.pose.position.z = head_pose[2] / 1000.0;
      tf2::Quaternion head_orientation;
      head_orientation.setRPY(-head_pose[3], head_pose[4] + M_PI, -head_pose[5]);
      msg_head_pose.pose.orientation = tf2::toMsg(head_orientation);

      pub_head_pose_->publish(msg_head_pose);

      if (this->get_parameter("broadcast_tf").get_value<bool>())
      {
        transform.child_frame_id = TF_FRAME_HEAD;
        transform.transform.translation.x = msg_head_pose.pose.position.x;
        transform.transform.translation.y = msg_head_pose.pose.position.y;
        transform.transform.translation.z = msg_head_pose.pose.position.z;
        transform.transform.rotation = msg_head_pose.pose.orientation;
        tf2_broadcaster_.sendTransform(transform);
      }

      if (this->get_parameter("visualise").get_value<bool>())
      {
        visualiser_.SetObservationPose(head_pose, face_landmark_detector_.detection_certainty);
      }
    }

    // Get gaze and/or eye features, if desired
    auto enable_eye_features = this->get_parameter("enable_eye_features").get_value<std::vector<bool>>();
    if (this->get_parameter("enable_gaze").get_value<bool>() || enable_eye_features[0] || enable_eye_features[1] || enable_eye_features[2])
    {
      // Compute eye landmarks
      std::vector<cv::Point2f> eye_landmarks = LandmarkDetector::CalculateAllEyeLandmarks(face_landmark_detector_);
      std::vector<cv::Point3f> eye_landmarks_3d = LandmarkDetector::Calculate3DEyeLandmarks(face_landmark_detector_, fx, fy, cx, cy);

      // Add eye features to the message, if desired
      if (enable_eye_features[0]) // 2D landmarks
      {
        openface_msgs::msg::PixelsStamped msg_eye_landmarks;
        msg_eye_landmarks.header = img->header;

        for (auto landmark : eye_landmarks)
        {
          openface_msgs::msg::Pixel landmark_px;
          landmark_px.x = landmark.x;
          landmark_px.y = landmark.y;
          msg_eye_landmarks.pixels.push_back(landmark_px);
        }

        pub_eye_landmarks_->publish(msg_eye_landmarks);
      }
      if (enable_eye_features[1]) // 2D visible landmarks
      {
        openface_msgs::msg::PixelsStamped msg_eye_landmarks_visible;
        msg_eye_landmarks_visible.header = img->header;

        std::vector<cv::Point2f> eye_landmarks_visible = LandmarkDetector::CalculateVisibleEyeLandmarks(face_landmark_detector_);
        for (auto landmark : eye_landmarks_visible)
        {
          openface_msgs::msg::Pixel landmark_px;
          landmark_px.x = landmark.x;
          landmark_px.y = landmark.y;
          msg_eye_landmarks_visible.pixels.push_back(landmark_px);
        }

        pub_eye_landmarks_visible_->publish(msg_eye_landmarks_visible);
      }
      if (enable_eye_features[2]) // 3D landmarks
      {
        openface_msgs::msg::PointsStamped msg_eye_landmarks_3d;
        msg_eye_landmarks_3d.header = img->header;

        for (auto landmark : eye_landmarks_3d)
        {
          geometry_msgs::msg::Point landmark_3d;
          landmark_3d.x = landmark.x / 1000.0;
          landmark_3d.y = landmark.y / 1000.0;
          landmark_3d.z = landmark.z / 1000.0;
          msg_eye_landmarks_3d.points.push_back(landmark_3d);
        }

        pub_landmarks_3d_->publish(msg_eye_landmarks_3d);
      }

      // Get gaze, if desired
      if (this->get_parameter("enable_gaze").get_value<bool>())
      {
        geometry_msgs::msg::PoseStamped msg_gaze_left, msg_gaze_right;
        geometry_msgs::msg::Vector3Stamped msg_gaze_compound;
        msg_gaze_left.header = img->header;
        msg_gaze_right.header = img->header;
        msg_gaze_compound.header = img->header;

        // Left pupil centre (position)
        cv::Point3f pupil_left(0, 0, 0);
        for (size_t i = 48; i < 56; ++i)
        {
          pupil_left += eye_landmarks_3d[i];
        }
        pupil_left /= 8000; // Average and conversion to metres
        msg_gaze_left.pose.position.x = pupil_left.x;
        msg_gaze_left.pose.position.y = pupil_left.y;
        msg_gaze_left.pose.position.z = pupil_left.z;

        // Right pupil centre (position)
        cv::Point3f pupil_right(0, 0, 0);
        for (size_t i = 20; i < 28; ++i)
        {
          pupil_right += eye_landmarks_3d[i];
        }
        pupil_right /= 8000; // Average and conversion to metres
        msg_gaze_right.pose.position.x = pupil_right.x;
        msg_gaze_right.pose.position.y = pupil_right.y;
        msg_gaze_right.pose.position.z = pupil_right.z;

        // Quat for flipping the eye gaze
        tf2::Quaternion q;
        q.setRPY(0, M_PI, M_PI);

        // Left eye gaze orientation
        cv::Point3f gaze_left(0, 0, -1);
        GazeAnalysis::EstimateGaze(face_landmark_detector_, gaze_left, fx, fy, cx, cy, true);
        tf2::Quaternion gaze_left_orientation;
        gaze_left_orientation.setValue(gaze_left.x, gaze_left.y, gaze_left.z);
        gaze_left_orientation *= q;
        msg_gaze_left.pose.orientation = tf2::toMsg(gaze_left_orientation);

        // Right eye gaze orientation
        cv::Point3f gaze_right(0, 0, -1);
        GazeAnalysis::EstimateGaze(face_landmark_detector_, gaze_right, fx, fy, cx, cy, false);
        tf2::Quaternion gaze_right_orientation;
        gaze_right_orientation.setValue(gaze_right.x, gaze_right.y, gaze_right.z);
        gaze_right_orientation *= q;
        msg_gaze_right.pose.orientation = tf2::toMsg(gaze_right_orientation);

        // Compund gaze orientation (average between left and right eye)
        cv::Vec2f gaze_compound = GazeAnalysis::GetGazeAngle(gaze_left, gaze_right);
        msg_gaze_compound.vector.x = gaze_compound[0];
        msg_gaze_compound.vector.y = gaze_compound[1];
        msg_gaze_compound.vector.z = 0;

        pub_gaze_left_->publish(msg_gaze_left);
        pub_gaze_right_->publish(msg_gaze_right);
        pub_gaze_compound_->publish(msg_gaze_compound);

        if (this->get_parameter("broadcast_tf").get_value<bool>())
        {
          transform.child_frame_id = TF_FRAME_EYE_LEFT;
          transform.transform.translation.x = msg_gaze_left.pose.position.x;
          transform.transform.translation.y = msg_gaze_left.pose.position.y;
          transform.transform.translation.z = msg_gaze_left.pose.position.z;
          transform.transform.rotation = msg_gaze_left.pose.orientation;
          tf2_broadcaster_.sendTransform(transform);

          transform.child_frame_id = TF_FRAME_EYE_RIGHT;
          transform.transform.translation.x = msg_gaze_right.pose.position.x;
          transform.transform.translation.y = msg_gaze_right.pose.position.y;
          transform.transform.translation.z = msg_gaze_right.pose.position.z;
          transform.transform.rotation = msg_gaze_right.pose.orientation;
          tf2_broadcaster_.sendTransform(transform);
        }

        if (this->get_parameter("visualise").get_value<bool>())
        {
          visualiser_.SetObservationGaze(gaze_left, gaze_right, eye_landmarks, eye_landmarks_3d, face_landmark_detector_.detection_certainty);
        }
      }
    }

    // Get Action Units, if desired
    if (this->get_parameter("enable_action_units").get_value<bool>())
    {
      openface_msgs::msg::ActionUnitsStamped msg_action_units;
      msg_action_units.header = img->header;

      face_analyser_.PredictStaticAUsAndComputeFeatures(cv_img_bgr8->image, face_landmark_detector_.detected_landmarks);
      auto aus_reg = face_analyser_.GetCurrentAUsReg();
      auto aus_class = face_analyser_.GetCurrentAUsClass();

      std::unordered_map<std::string, openface_msgs::msg::ActionUnit> aus;
      for (const auto &au_reg : aus_reg)
      {
        auto it = aus.find(std::get<0>(au_reg));
        if (it == aus.end())
        {
          openface_msgs::msg::ActionUnit u;
          u.name = std::get<0>(au_reg);
          u.intensity = std::get<1>(au_reg);
          aus.insert({std::get<0>(au_reg), u});
          continue;
        }
        it->second.intensity = std::get<1>(au_reg);
      }

      for (const auto &au_class : aus_class)
      {
        auto it = aus.find(std::get<0>(au_class));
        if (it == aus.end())
        {
          openface_msgs::msg::ActionUnit u;
          u.name = std::get<0>(au_class);
          u.presence = std::get<1>(au_class);
          aus.insert({std::get<0>(au_class), u});
          continue;
        }
        it->second.presence = std::get<1>(au_class);
      }

      for (const auto &au : aus)
      {
        msg_action_units.action_units.push_back(std::get<1>(au));
      }

      pub_action_units_->publish(msg_action_units);

      if (this->get_parameter("visualise").get_value<bool>())
      {
        visualiser_.SetObservationActionUnits(aus_reg, aus_class);
      }
    }
  }

  // Aligned face - Only for visualisation
  if (this->get_parameter("enable_aligned_face").get_value<bool>())
  {
    cv::Mat sim_warped_img;
    face_analyser_.GetLatestAlignedFace(sim_warped_img);
    if (this->get_parameter("visualise").get_value<bool>())
    {
      visualiser_.SetObservationFaceAlign(sim_warped_img);
    }
  }

  // Facial HOG - Only for visualisation
  if (this->get_parameter("enable_hog").get_value<bool>())
  {
    cv::Mat_<double> hog_descriptor;
    int num_hog_rows = 0, num_hog_cols = 0;
    face_analyser_.GetLatestHOG(hog_descriptor, num_hog_rows, num_hog_cols);
    if (this->get_parameter("visualise").get_value<bool>())
    {
      visualiser_.SetObservationHOG(hog_descriptor, num_hog_rows, num_hog_cols);
    }
  }

  // Visualise, if desired
  if (this->get_parameter("visualise").get_value<bool>())
  {
    if (this->get_parameter("track_fps").get_value<bool>())
    {
      visualiser_.SetFps(fps_tracker_.GetFPS());
    }
    visualiser_.ShowObservation();
    cv::waitKey(1);
  }
  else if (this->get_parameter("track_fps").get_value<bool>())
  {
    // If FPS tracking is enabled without visualisation, log FPS to RCLCPP_DEBUG
    RCLCPP_DEBUG(this->get_logger(), "FPS: %f", fps_tracker_.GetFPS());
  }
}

////////////
/// MAIN ///
////////////

/// Main function that initiates an object of `Ros2OpenFaceSeparate` class as the core of this node.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2OpenFaceSeparate>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
