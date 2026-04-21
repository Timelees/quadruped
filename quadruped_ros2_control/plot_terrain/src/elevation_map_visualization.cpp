#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <convex_plane_decomposition/LoadGridmapFromImage.h>
#include <memory>

#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>

#include "convex_plane_decomposition_ros/MessageConversion.h"
#include "convex_plane_decomposition_ros/ParameterLoading.h"
#include "convex_plane_decomposition_ros/RosVisualizations.h"

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <convex_plane_decomposition/Timer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>

#include <grid_map_filters_rsl/inpainting.hpp>
#include <grid_map_filters_rsl/smoothing.hpp>
#include <convex_plane_decomposition/Timer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <grid_map_cv/GridMapCvProcessing.hpp>
#include <opencv2/core/eigen.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp> 
namespace grid_map {
GridMap getTransformedMap(GridMap&& gridMapSource,
                          const Eigen::Isometry3d& transform,
                          const std::string& heightLayerName,
                          const std::string& newFrameId) {
  // Check if height layer is valid.
  if (!gridMapSource.exists(heightLayerName)) {
    throw std::out_of_range("GridMap::getTransformedMap(...) : No map layer '" +
                            heightLayerName + "' available.");
  }

  // Check if transformation is z aligned.
  if (std::abs(transform(2, 2) - 1) >= 0.05) {
    throw std::invalid_argument("The given transform is not Z aligned!");
  }

  auto yawPitchRoll =
      transform.rotation().eulerAngles(2, 1, 0);  // Double check convention!
  double rotationAngle = yawPitchRoll.x() * 180 / CV_PI;
  if (std::abs(yawPitchRoll.y()) >= 3 &&
      std::abs(yawPitchRoll.z()) >=
          3) {  // Resolve yaw ambiguity in euler angles.
    rotationAngle += 180;
  }

  gridMapSource.convertToDefaultStartIndex();

  // Create the rotated gridMap.
  GridMap transformedMap(gridMapSource.getLayers());
  transformedMap.setBasicLayers(gridMapSource.getBasicLayers());
  transformedMap.setTimestamp(gridMapSource.getTimestamp());
  transformedMap.setFrameId(newFrameId);

  // openCV rotation parameters, initalized on first layer.
  cv::Mat imageRotationMatrix;
  cv::Rect2f boundingBox;

  bool firstLayer = true;
  for (const auto& layer : gridMapSource.getLayers()) {
    cv::Mat imageSource, imageResult;

    // From gridMap to openCV image. Assumes defaultStartIndex.
    cv::eigen2cv(gridMapSource[layer], imageSource);

    // Calculate transformation matrix and update geometry of the resulting grid
    // map.
    if (firstLayer) {
      // Get rotation matrix for rotating the image around its center in pixel
      // coordinates. See
      // https://answers.opencv.org/question/82708/rotation-center-confusion/
      cv::Point2f imageCenter = cv::Point2f((imageSource.cols - 1) / 2.0,
                                            (imageSource.rows - 1) / 2.0);

      imageRotationMatrix =
          cv::getRotationMatrix2D(imageCenter, rotationAngle, 1.0);
      boundingBox =
          cv::RotatedRect(cv::Point2f(0, 0), imageSource.size(), rotationAngle)
              .boundingRect2f();

      // Adjust transformation matrix. See
      // https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#gafbbc470ce83812914a70abfb604f4326
      // and
      // https://stackoverflow.com/questions/22041699/rotate-an-image-without-cropping-in-opencv-in-c
      imageRotationMatrix.at<double>(0, 2) +=
          boundingBox.width / 2.0 - imageSource.cols / 2.0;
      imageRotationMatrix.at<double>(1, 2) +=
          boundingBox.height / 2.0 - imageSource.rows / 2.0;

      // Calculate the new center of the gridMap.
      Position3 newCenter =
          transform * Position3{(gridMapSource.getPosition().x()),
                                (gridMapSource.getPosition().y()), 0.0};

      // Set the size of the rotated gridMap.
      transformedMap.setGeometry(
          {boundingBox.height * gridMapSource.getResolution(),
           boundingBox.width * gridMapSource.getResolution()},
          gridMapSource.getResolution(),
          Position(newCenter.x(), newCenter.y()));
      firstLayer = false;
    }

    // Rotate the layer.
    imageResult = cv::Mat(boundingBox.size(), CV_32F,
                          std::numeric_limits<double>::quiet_NaN());
    cv::warpAffine(imageSource, imageResult, imageRotationMatrix,
                   boundingBox.size(), cv::INTER_NEAREST,
                   cv::BORDER_TRANSPARENT);

    // Copy result into gridMapLayer. Assumes default start index.
    Matrix resultLayer;
    cv::cv2eigen(imageResult, resultLayer);
    transformedMap.add(layer, resultLayer);
  }

  // Add height translation.
  grid_map::Matrix heightLayer = transformedMap[heightLayerName];
  transformedMap[heightLayerName] =
      heightLayer.array() + transform.translation().z();

  return transformedMap;
}
}  // namespace grid_map


// 全局变量用于存储位置数据

double positionx = 0;
double positiony = 0;
double positionz = 0;


// 回调函数，当接收到消息时被调用
void odomCallback(const std::shared_ptr<const nav_msgs::msg::Odometry>& msg)
{
  // ROS_INFO("ground truth:\n");
  // ROS_INFO("ground truth: [%.2f, %.2f]",
  //          msg->pose.pose.position.x,
  //          msg->pose.pose.position.y);
  // 提取所有数据
  positionx = msg->pose.pose.position.x;
  positiony = msg->pose.pose.position.y;
  positionz = msg->pose.pose.position.z;
}

// Noise
double noiseUniform;
double noiseGauss;
double outlierPercentage;
bool blur;
grid_map::GridMap::Matrix noiseLayer;


void createNoise(size_t row, size_t col)
{
  // Box-Muller Transform
  grid_map::GridMap::Matrix u1 = 0.5f * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5f;
  grid_map::GridMap::Matrix u2 = 0.5f * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5f;
  grid_map::GridMap::Matrix gauss01 =
      u1.binaryExpr(u2, [&](float v1, float v2) -> float
                    { return std::sqrt(-2.0f * log(v1)) * cos(2.0f * M_PIf32 * v2); });

  noiseLayer = (noiseUniform * grid_map::GridMap::Matrix::Random(row, col) + noiseGauss * gauss01).cast<float>();
}

// Parameters
std::string elevationMapTopic_("/elevation_mapping/elevation_map_raw");
std::string elevationLayer_("elevation");
std::string targetFrameId_("odom");
double subMapWidth_;
double subMapLength_;
bool publishToController_;
double resolution_;
std::unique_ptr<convex_plane_decomposition::PlaneDecompositionPipeline> planeDecompositionPipeline_;
std::shared_ptr<tf2_ros::Buffer> tfBuffer;

bool loadParameters(const rclcpp::Node::SharedPtr nodeHandle)
{
  if (!nodeHandle->get_parameter("elevation_topic", elevationMapTopic_))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] Could not read parameter `elevation_topic`.");
    return false;
  }
  if (!nodeHandle->get_parameter("target_frame_id", targetFrameId_))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] Could not read parameter `target_frame_id`.");
    return false;
  }
  if (!nodeHandle->get_parameter("height_layer", elevationLayer_))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] Could not read parameter `height_layer`.");
    return false;
  }
  if (!nodeHandle->get_parameter("submap.width", subMapWidth_))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] Could not read parameter `submap.width`.");
    return false;
  }
  if (!nodeHandle->get_parameter("submap.length", subMapLength_))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] Could not read parameter `submap.length`.");
    return false;
  }
  if (!nodeHandle->get_parameter("publish_to_controller", publishToController_))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] Could not read parameter `publish_to_controller`.");
    return false;
  }
  if (!nodeHandle->get_parameter("noiseGauss", noiseGauss))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `noiseGauss`.");
    return false;
  }
  if (!nodeHandle->get_parameter("noiseUniform", noiseUniform))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `noiseUniform`.");
    return false;
  }
  if (!nodeHandle->get_parameter("blur", blur))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `blur`.");
    return false;
  }
  if (!nodeHandle->get_parameter("outlier_percentage", outlierPercentage))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `outlier_percentage`.");
    return false;
  }
  if(!nodeHandle->get_parameter("resolution", resolution_))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `resolution`.");
    return false;
  }


  convex_plane_decomposition::PlaneDecompositionPipeline::Config config;

  config.preprocessingParameters = convex_plane_decomposition::loadPreprocessingParameters(nodeHandle.get(), "preprocessing.");
  config.contourExtractionParameters = convex_plane_decomposition::loadContourExtractionParameters(nodeHandle.get(), "contour_extraction.");
  config.ransacPlaneExtractorParameters = convex_plane_decomposition::loadRansacPlaneExtractorParameters(nodeHandle.get(), "ransac_plane_refinement.");
  config.slidingWindowPlaneExtractorParameters = convex_plane_decomposition::loadSlidingWindowPlaneExtractorParameters(nodeHandle.get(), "sliding_window_plane_extractor.");
  config.postprocessingParameters = convex_plane_decomposition::loadPostprocessingParameters(nodeHandle.get(), "postprocessing.");

  planeDecompositionPipeline_ = std::make_unique<convex_plane_decomposition::PlaneDecompositionPipeline>(config);
  resolution_ = config.preprocessingParameters.resolution;
  return true;
}


// ros发布订阅
rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr elevationMapSubscriber_;
rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr filteredmapPublisher_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundaryPublisher_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr insetPublisher_;
rclcpp::Publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr regionPublisher_;
rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr elevationmapPublisher_;

Eigen::Isometry3d getTransformToTargetFrame(rclcpp::Node* nodeHandle, const std::string& sourceFrame, const rclcpp::Time& time) {
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer->lookupTransform(targetFrameId_, sourceFrame, time);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] %s", ex.what());
    return Eigen::Isometry3d::Identity();
  }

  Eigen::Isometry3d transformation;

  // Extract translation.
  transformation.translation().x() = transformStamped.transform.translation.x;
  transformation.translation().y() = transformStamped.transform.translation.y;
  transformation.translation().z() = transformStamped.transform.translation.z;

  // Extract rotation.
  Eigen::Quaterniond rotationQuaternion(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                                        transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
  transformation.linear() = rotationQuaternion.toRotationMatrix();
  return transformation;
}

int main(int argc, char* argv[]) {

  const std::string terrainFolder = ament_index_cpp::get_package_share_directory("plot_terrain") + "/data/";
  std::cout << "terrainFolder: " << terrainFolder << std::endl;

  rclcpp::init(argc, argv);
  auto nodeHandle = rclcpp::Node::make_shared("plot_terrain_ConvexPlaneDecomposition",
                                              rclcpp::NodeOptions().allow_undeclared_parameters(true)
                                              .automatically_declare_parameters_from_overrides(true));

  // Load parameters
  std::string terrainFile{"stepping_stones_90.png"};
  nodeHandle->get_parameter("terrain_name", terrainFile);
  std::cout << "terrainFile: " << terrainFile << std::endl;

  double heightScale{1.0};
  nodeHandle->get_parameter("terrain_scale", heightScale);
  double resolution{0.04};
  nodeHandle->get_parameter("resolution", resolution);

  // Initialize publisher
  auto elevationmapPublisher_ = nodeHandle->create_publisher<grid_map_msgs::msg::GridMap>("/elevation_mapping/elevation_map_raw", 1);

  // Load grid map from image
  const std::string frameId{"world"};
  auto gridMap = convex_plane_decomposition::loadGridmapFromImage(terrainFolder + "/" + terrainFile, "elevation", frameId, resolution, heightScale);
  // Set a valid timestamp for the grid map
  gridMap.setTimestamp(nodeHandle->now().nanoseconds());

  RCLCPP_INFO(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] Loaded map from image with size %f x %f m (%i x %i cells).",
              gridMap.getLength().x(), gridMap.getLength().y(), gridMap.getSize()(0), gridMap.getSize()(1));

  // Publish the elevation map
  auto rawOutputMessage = grid_map::GridMapRosConverter::toMessage(gridMap);
  elevationmapPublisher_->publish(*rawOutputMessage);

  // Keep the node alive to allow RViz to receive the message
  rclcpp::Rate rate(1.0); // Publish once per second
  while (rclcpp::ok()) {
    gridMap.setTimestamp(nodeHandle->now().nanoseconds());
    elevationmapPublisher_->publish(*rawOutputMessage);
    rclcpp::spin_some(nodeHandle);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}