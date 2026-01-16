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
namespace grid_map
{
  GridMap getTransformedMap(GridMap &&gridMapSource,
                            const Eigen::Isometry3d &transform,
                            const std::string &heightLayerName,
                            const std::string &newFrameId)
  {
    // Check if height layer is valid.
    if (!gridMapSource.exists(heightLayerName))
    {
      throw std::out_of_range("GridMap::getTransformedMap(...) : No map layer '" +
                              heightLayerName + "' available.");
    }

    // Check if transformation is z aligned.
    if (std::abs(transform(2, 2) - 1) >= 0.05)
    {
      throw std::invalid_argument("The given transform is not Z aligned!");
    }

    auto yawPitchRoll =
        transform.rotation().eulerAngles(2, 1, 0); // Double check convention!
    double rotationAngle = yawPitchRoll.x() * 180 / CV_PI;
    if (std::abs(yawPitchRoll.y()) >= 3 &&
        std::abs(yawPitchRoll.z()) >=
            3)
    { // Resolve yaw ambiguity in euler angles.
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
    for (const auto &layer : gridMapSource.getLayers())
    {
      cv::Mat imageSource, imageResult;

      // From gridMap to openCV image. Assumes defaultStartIndex.
      cv::eigen2cv(gridMapSource[layer], imageSource);

      // Calculate transformation matrix and update geometry of the resulting grid
      // map.
      if (firstLayer)
      {
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
} // namespace grid_map

// 全局变量用于存储位置数据
double positionx = 0;
double positiony = 0;
double positionz = 0;

// 标记是否已经将地图从world坐标系转换到目标坐标系
bool mapTransformed = false;

// 回调函数，当接收到消息时被调用
void odomCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
{
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
  if (!nodeHandle->get_parameter("resolution", resolution_))
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

Eigen::Isometry3d getTransformToTargetFrame(rclcpp::Node *nodeHandle, const std::string &sourceFrame, const rclcpp::Time &time)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    // 使用较短的超时时间，避免阻塞
    transformStamped =
        tfBuffer->lookupTransform(targetFrameId_, sourceFrame, time, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] TF lookup failed (%s -> %s): %s",
                 sourceFrame.c_str(), targetFrameId_.c_str(), ex.what());
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

int main(int argc, char *argv[])
{
  const std::string terrainFolder = ament_index_cpp::get_package_share_directory("plot_terrain") + "/data/";
  const std::string robotName = "robot";

  rclcpp::init(argc, argv);
  auto nodeHandle = rclcpp::Node::make_shared("plot_terrain_ConvexPlaneDecomposition",
                                              rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
  tfBuffer = std::make_shared<tf2_ros::Buffer>(nodeHandle->get_clock());

  std::string pubName("planar_terrain");
  nodeHandle->get_parameter("pubTopicName", pubName);

  RCLCPP_INFO(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] planar_terrain publish Topic Name is  %s", pubName.c_str());
  RCLCPP_INFO(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] elevationMapTopic_ Topic Name is  %s", elevationMapTopic_.c_str());

  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub = nodeHandle->create_subscription<nav_msgs::msg::Odometry>(
  //   "/ground_truth/state", 1000, odomCallback);

  tf2_ros::TransformListener tfListener(*tfBuffer);
  static tf2_ros::TransformBroadcaster br(nodeHandle);

  bool parametersLoaded = loadParameters(nodeHandle);

  if (parametersLoaded)
  {
    filteredmapPublisher_ = nodeHandle->create_publisher<grid_map_msgs::msg::GridMap>("filtered_map", 1);
    boundaryPublisher_ = nodeHandle->create_publisher<visualization_msgs::msg::MarkerArray>("boundaries", 1);
    insetPublisher_ = nodeHandle->create_publisher<visualization_msgs::msg::MarkerArray>("insets", 1);
    regionPublisher_ = nodeHandle->create_publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>(pubName, 1);
    elevationmapPublisher_ = nodeHandle->create_publisher<grid_map_msgs::msg::GridMap>(elevationMapTopic_, 1);
  }

  const std::string frameId{"world"};
  bool debug = false;
  std::string terrainFile{"stepping_stones_90.png"};
  nodeHandle->get_parameter("terrain_name", terrainFile);
  // std::cout << "terrainFile: " << terrainFile << std::endl;
  nodeHandle->get_parameter("debug", debug);

  double heightScale{1.0};
  nodeHandle->get_parameter("terrain_scale", heightScale);
  // 获取高程图
  auto gridMap = convex_plane_decomposition::loadGridmapFromImage(terrainFolder + "/" + terrainFile, elevationLayer_, frameId,
                                                                  resolution_, heightScale);
  
  //检查高度图是否包含有效值。
  if (!convex_plane_decomposition::containsFiniteValue(gridMap.get(elevationLayer_))) { 
    RCLCPP_WARN(nodeHandle->get_logger(),
                "[ConvexPlaneExtractionROS] map does not contain any values");
    return 1;
  }
  // 添加噪声
  if (blur)
  {
    // Copy!
    auto originalMap = gridMap.get(elevationLayer_);

    // Blur (remove nan -> filter -> put back nan
    grid_map::inpainting::minValues(gridMap, elevationLayer_, "i");
    grid_map::smoothing::boxBlur(gridMap, "i", elevationLayer_, 3, 1);
    gridMap.get(elevationLayer_) = (originalMap.array().isFinite()).select(gridMap.get(elevationLayer_), originalMap);

    auto &elevation = gridMap.get(elevationLayer_);
    if (noiseLayer.size() != elevation.size())
    {
      createNoise(elevation.rows(), elevation.cols());
    }

    elevation += noiseLayer;
    gridMap[elevationLayer_] = elevation;
  }

  RCLCPP_INFO(nodeHandle->get_logger(), "[ConvexPlaneExtractionROS] Loaded map from image with size %f x %f m (%i x %i cells).",
              gridMap.getLength().x(), gridMap.getLength().y(), gridMap.getSize()(0), gridMap.getSize()(1));
  RCLCPP_INFO(nodeHandle->get_logger(), "gridMap it has this layers: \n");
  std::vector<std::string> layers;
  layers = gridMap.getLayers();
  for (const auto &layer : layers)
  {
    RCLCPP_INFO(nodeHandle->get_logger(), "- %s", layer.c_str());
  }
  double frequency = 20.0;

  if (!nodeHandle->get_parameter("frequency", frequency))
  {
    RCLCPP_ERROR(nodeHandle->get_logger(), "[ConvexPlaneDecompositionNode] Could not read parameter `frequency`.");
    return 1;
  }
  bool isprint = 0;
  rclcpp::Rate rate(frequency);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nodeHandle);

  // 等待TF树可用
  RCLCPP_INFO(nodeHandle->get_logger(), "[ConvexPlaneDecomposition] Waiting for TF tree (odom -> base) to become available...");
  while (rclcpp::ok())
  {
    executor.spin_some();
    if (tfBuffer->canTransform("odom", "base", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)))
    {
      RCLCPP_INFO(nodeHandle->get_logger(), "[ConvexPlaneDecomposition] TF tree available, starting terrain processing.");
      break;
    }
    RCLCPP_INFO_THROTTLE(nodeHandle->get_logger(), *nodeHandle->get_clock(), 5000,
        "[ConvexPlaneDecomposition] Still waiting for TF (odom -> base)...");
    rate.sleep();
  }

  while (rclcpp::ok())
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      // Lookup the transform from "odom" to "base"
      std::string errorMsg;
      
      // 发布原始高程图数据
      auto rawOutputMessage = grid_map::GridMapRosConverter::toMessage(gridMap);
      elevationmapPublisher_->publish(*rawOutputMessage);

      rclcpp::Time timeStamp = rclcpp::Time(0); // Use Time(0) to get the latest transform.

      // 检查TF是否可用 - 使用正确的frame名称
      // Gazebo仿真中: odom -> base (通过ros_gz_bridge发布)
      if (tfBuffer->canTransform("odom", "base", timeStamp, rclcpp::Duration::from_seconds(0.5)))
      {
        transformStamped = tfBuffer->lookupTransform("odom", "base", rclcpp::Time(0));

        // 地图坐标系转换：只需要做一次（world -> odom）
        // 由于 world 和 odom 之间是单位变换，实际上只需要确保frame_id正确
        if (!mapTransformed && gridMap.getFrameId() != targetFrameId_)
        {
          if (tfBuffer->canTransform(targetFrameId_, gridMap.getFrameId(), timeStamp, rclcpp::Duration::from_seconds(1.0)))
          {
            const auto transform = getTransformToTargetFrame(nodeHandle.get(), gridMap.getFrameId(), timeStamp);
            gridMap = grid_map::getTransformedMap(std::move(gridMap), transform, elevationLayer_, targetFrameId_);
            mapTransformed = true;
            RCLCPP_INFO(nodeHandle->get_logger(), "[ConvexPlaneDecomposition] Map transformed from '%s' to '%s'",
                        gridMap.getFrameId().c_str(), targetFrameId_.c_str());
          }
          else
          {
            // 如果world->odom变换不可用，直接设置frame_id（假设是单位变换）
            RCLCPP_WARN_ONCE(nodeHandle->get_logger(), 
                "[ConvexPlaneDecomposition] TF from '%s' to '%s' not available, assuming identity transform",
                gridMap.getFrameId().c_str(), targetFrameId_.c_str());
            gridMap.setFrameId(targetFrameId_);
            mapTransformed = true;
          }
        }

        grid_map::Position submapPosition(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
        // grid_map::Position submapPosition(positionx, positiony);
        if (debug)
          RCLCPP_INFO(nodeHandle->get_logger(), "Translation: [%.2f, %.2f]",
                      transformStamped.transform.translation.x,
                      transformStamped.transform.translation.y);

        // 根据机器人当前位置提取子地图
        bool success;
        grid_map::GridMap elevationMap = gridMap.getSubmap(submapPosition, Eigen::Array2d(subMapLength_, subMapWidth_), success);
        if (debug)
          RCLCPP_INFO(nodeHandle->get_logger(), "getSubmap it has this layers");

        const grid_map::Matrix elevationRaw = elevationMap.get(elevationLayer_);

        
        // Run pipeline.对子图进行平面分解，获得地形平面区域
        planeDecompositionPipeline_->update(std::move(elevationMap), elevationLayer_);
        auto &planarTerrain = planeDecompositionPipeline_->getPlanarTerrain();
        if (publishToController_)
        {
          regionPublisher_->publish(toMessage(planarTerrain));
        }

        // Add raw map
        planarTerrain.gridMap.add("elevation_raw", elevationRaw);

        // Add segmentation
        planarTerrain.gridMap.add("segmentation");
        planeDecompositionPipeline_->getSegmentation(planarTerrain.gridMap.get("segmentation"));
        if (debug)
        {
          RCLCPP_INFO(nodeHandle->get_logger(), "planarTerrain.gridMap it has this layers");
          layers = planarTerrain.gridMap.getLayers();
          for (const auto &layer : layers)
          {
            RCLCPP_INFO(nodeHandle->get_logger(), "- %s", layer.c_str());
          }
        }
        grid_map_msgs::msg::GridMap outputMessage;
        outputMessage = *(grid_map::GridMapRosConverter::toMessage(planarTerrain.gridMap));
        filteredmapPublisher_->publish(outputMessage);

        const double lineWidth = 0.005; // [m] RViz marker size
        boundaryPublisher_->publish(convertBoundariesToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                                  planarTerrain.gridMap.getTimestamp(), lineWidth));
        insetPublisher_->publish(convertInsetsToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                           planarTerrain.gridMap.getTimestamp(), lineWidth));

        // iter++;

        // if (iter % 40 == 0)
        // {

        //   std::stringstream infoStream;
        //   infoStream << "\n########################################################################\n";
        //   infoStream << "The benchmarking is computed over " << iter << " iterations. \n";
        //   infoStream << "PlaneExtraction Benchmarking    : Average time [ms], Max time [ms]\n";
        //   auto printLine = [](std::string name, const convex_plane_decomposition::Timer &timer)
        //   {
        //     std::stringstream ss;
        //     ss << std::fixed << std::setprecision(2);
        //     ss << "\t" << name << "\t: " << std::setw(17) << timer.getAverageInMilliseconds() << ", " << std::setw(13)
        //        << timer.getMaxIntervalInMilliseconds() << "\n";
        //     return ss.str();
        //   };
        //   infoStream << printLine("Pre-process        ", planeDecompositionPipeline_->getPrepocessTimer());
        //   infoStream << printLine("Sliding window     ", planeDecompositionPipeline_->getSlidingWindowTimer());
        //   infoStream << printLine("Contour extraction ", planeDecompositionPipeline_->getContourExtractionTimer());
        //   infoStream << printLine("Post-process       ", planeDecompositionPipeline_->getPostprocessTimer());
        //   std::cerr << infoStream.str() << std::endl;
        // }
        if (isprint)
        {
          RCLCPP_INFO(nodeHandle->get_logger(), "Translation: [%.2f, %.2f, %.2f]",
                      transformStamped.transform.translation.x,
                      transformStamped.transform.translation.y,
                      transformStamped.transform.translation.z);
          // ROS_INFO("Rotation: [%.2f, %.2f, %.2f, %.2f]",
          //          transformStamped.transform.rotation.x,
          //          transformStamped.transform.rotation.y,
          //          transformStamped.transform.rotation.z,
          //          transformStamped.transform.rotation.w);

          RCLCPP_INFO(nodeHandle->get_logger(), "ground truth: [%.2f, %.2f]",
                      positionx,
                      positiony);
        }
      }
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(nodeHandle->get_logger(), "Failed to lookup transform: %s", ex.what());
    }

    executor.spin_some();
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}