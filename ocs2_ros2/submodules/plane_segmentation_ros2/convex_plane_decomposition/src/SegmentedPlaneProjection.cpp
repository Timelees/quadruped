//
// Created by rgrandia on 12.10.21.
//

#include "convex_plane_decomposition/SegmentedPlaneProjection.h"

#include "convex_plane_decomposition/GeometryUtils.h"

namespace convex_plane_decomposition {

namespace {  // Helper functions that only make sense in this context

// 计算两点之间的欧几里得距离平方。用于评估投影点与原始查询点的接近程度。
double distanceCost(const Eigen::Vector3d& query, const Eigen::Vector3d& terrainPoint) {
  const double dx = query.x() - terrainPoint.x();
  const double dy = query.y() - terrainPoint.y();
  const double dz = query.z() - terrainPoint.z();
  return dx * dx + dy * dy + dz * dz;
}

// 直接返回输入的距离平方，作为下界判断的保守估计。在排序后遍历区域时，若某个区域的边界框距离下界已经大于当前最优代价，则可提前终止搜索。
double distanceCostLowerbound(double distanceSquared) {
  return distanceSquared;
}
// 给定一个值 value 和一个区间 [min, max]，返回 value 到该区间的距离平方
double intervalSquareDistance(double value, double min, double max) {
  //   -    |    0     |    +
  //       min        max
  // Returns 0.0 if between min and max. Returns the distance to one boundary otherwise.
  if (value < min) {
    double diff = min - value;
    return diff * diff;
  } else if (value < max) {
    return 0.0;
  } else {
    double diff = max - value;
    return diff * diff;
  }
}

// 计算一个二维点（CgalPoint2d）到一个轴对齐包围盒（CgalBbox2d）的最近距离平方。它分别在 x 和 y 方向上调用 intervalSquareDistance，然后求和
double squaredDistanceToBoundingBox(const CgalPoint2d& point, const CgalBbox2d& boundingBox) {
  const double dxdx = intervalSquareDistance(point.x(), boundingBox.xmin(), boundingBox.xmax());
  const double dydy = intervalSquareDistance(point.y(), boundingBox.ymin(), boundingBox.ymax());
  return dxdx + dydy;
}
// 给定一个二维点，在一组孔洞（insets）中查找包含该点的第一个孔洞。孔洞是多边形带孔区域中的“禁止区域”，即不能落足的部分。
const CgalPolygonWithHoles2d* findInsetContainingThePoint(const CgalPoint2d& point, const std::vector<CgalPolygonWithHoles2d>& insets) {
  for (const auto& inset : insets) {
    if (isInside(point, inset.outer_boundary())) {
      return &inset;
    }
  }
  return nullptr;
}

}  // namespace

// 将一个已经投影到平面坐标系中的点（即查询点在平面上的垂直投影）进一步投影到该平面区域的可落足范围内。
CgalPoint2d projectToPlanarRegion(const CgalPoint2d& queryProjectedToPlane, const PlanarRegion& planarRegion) {
  // First search if the projected point is inside any of the insets.
  // Note: we know that all insets are non-overlapping, and are not nested (no shape is contained in the hole of another shape)
  const auto* const insetPtrContainingPoint = findInsetContainingThePoint(queryProjectedToPlane, planarRegion.boundaryWithInset.insets);

  // Compute the projection 计算投影
  CgalPoint2d projectedPoint;
  if (insetPtrContainingPoint == nullptr) {
    // Not inside any of the insets. Need to look for the closest one. The projection will be to the boundary
    double minDistSquared = std::numeric_limits<double>::max();
    for (const auto& inset : planarRegion.boundaryWithInset.insets) {
      const auto closestPoint = projectToClosestPoint(queryProjectedToPlane, inset.outer_boundary());
      double distSquare = squaredDistance(closestPoint, queryProjectedToPlane);
      if (distSquare < minDistSquared) {
        projectedPoint = closestPoint;
        minDistSquared = distSquare;
      }
    }
  } else {
    // Query point is inside and does not need projection...
    projectedPoint = queryProjectedToPlane;

    // ... unless it is inside a hole
    for (const auto& hole : insetPtrContainingPoint->holes()) {
      if (isInside(queryProjectedToPlane, hole)) {
        projectedPoint = projectToClosestPoint(queryProjectedToPlane, hole);
        break;  // No need to search other holes. Holes are not overlapping
      }
    }
  }

  return projectedPoint;
}
// 给定一个世界坐标系下的点 positionInWorld 和一系列平面区域，为每个区域计算一个粗略的距离下界（基于其二维包围盒和垂直高度差），并按该下界从小到大排序，返回排序后的信息结构体数组。
std::vector<RegionSortingInfo> sortWithBoundingBoxes(const Eigen::Vector3d& positionInWorld,
                                                     const std::vector<convex_plane_decomposition::PlanarRegion>& planarRegions) {
  // Compute distance to bounding boxes
  std::vector<RegionSortingInfo> regionsAndBboxSquareDistances;
  regionsAndBboxSquareDistances.reserve(planarRegions.size());
  for (const auto& planarRegion : planarRegions) {
    const Eigen::Vector3d positionInTerrainFrame = planarRegion.transformPlaneToWorld.inverse() * positionInWorld;
    const double dzdz = positionInTerrainFrame.z() * positionInTerrainFrame.z();

    RegionSortingInfo regionSortingInfo;
    regionSortingInfo.regionPtr = &planarRegion;
    regionSortingInfo.positionInTerrainFrame = {positionInTerrainFrame.x(), positionInTerrainFrame.y()};
    regionSortingInfo.boundingBoxSquareDistance =
        squaredDistanceToBoundingBox(regionSortingInfo.positionInTerrainFrame, planarRegion.bbox2d) + dzdz;

    regionsAndBboxSquareDistances.push_back(regionSortingInfo);
  }

  // Sort regions close to far
  std::sort(regionsAndBboxSquareDistances.begin(), regionsAndBboxSquareDistances.end(),
            [](const RegionSortingInfo& lhs, const RegionSortingInfo& rhs) {
              return lhs.boundingBoxSquareDistance < rhs.boundingBoxSquareDistance;
            });

  return regionsAndBboxSquareDistances;
}
// 在世界坐标系下，给定一个点 positionInWorld，在众多平面区域中寻找最佳投影点，使得投影点落在区域可行部分（考虑孔洞），且满足一个代价函数最小化。
PlanarTerrainProjection getBestPlanarRegionAtPositionInWorld(const Eigen::Vector3d& positionInWorld,    // 查询点（名义落足点）在世界坐标系下的坐标
                                                             const std::vector<PlanarRegion>& planarRegions,  // 可选的平面区域列表
                                                             const std::function<double(const Eigen::Vector3d&)>& penaltyFunction) {  // 惩罚函数
  const auto sortedRegions = sortWithBoundingBoxes(positionInWorld, planarRegions);

  // Look for closest planar region.
  PlanarTerrainProjection projection;
  projection.cost = std::numeric_limits<double>::max();
  for (const auto& regionInfo : sortedRegions) {
    // Skip based on lower bound
    if (distanceCostLowerbound(regionInfo.boundingBoxSquareDistance) > projection.cost) {
      continue;
    }

    // Project onto planar region
    const auto projectedPointInTerrainFrame = projectToPlanarRegion(regionInfo.positionInTerrainFrame, *regionInfo.regionPtr);

    // Express projected point in World frame
    const auto projectionInWorldFrame =
        positionInWorldFrameFromPosition2dInPlane(projectedPointInTerrainFrame, regionInfo.regionPtr->transformPlaneToWorld);

    const auto cost = distanceCost(positionInWorld, projectionInWorldFrame) + penaltyFunction(projectionInWorldFrame);



    // std::cout << std::fixed << std::setprecision(3) << "[convex_plane_decomposition] Projection cost: " << cost << " =  (distance: " << distanceCost(positionInWorld, projectionInWorldFrame)
    //           << " + penalty: " << penaltyFunction(projectionInWorldFrame) << ")" << std::endl;

    if (cost < projection.cost) {
      projection.cost = cost;
      projection.regionPtr = regionInfo.regionPtr;
      projection.positionInTerrainFrame = projectedPointInTerrainFrame;
      projection.positionInWorld = projectionInWorldFrame;
    }
  }


  return projection;
}

}  // namespace convex_plane_decomposition