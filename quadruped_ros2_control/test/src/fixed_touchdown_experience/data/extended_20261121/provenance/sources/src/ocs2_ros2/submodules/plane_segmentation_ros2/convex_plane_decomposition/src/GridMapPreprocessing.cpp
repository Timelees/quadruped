#include "convex_plane_decomposition/GridMapPreprocessing.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <grid_map_filters_rsl/inpainting.hpp>
#include <grid_map_filters_rsl/smoothing.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <limits>

namespace convex_plane_decomposition {

GridMapPreprocessing::GridMapPreprocessing(const PreprocessingParameters& parameters) : parameters_(parameters) {}

void GridMapPreprocessing::preprocess(grid_map::GridMap& gridMap, const std::string& layer) const {
  const auto observed = gridMap;
  inpaint(gridMap, layer);
  denoise(gridMap, layer);
  changeResolution(gridMap, layer);

  // Keep observation evidence before either inpainting or median/resolution
  // filtering. The dense planning map and the certification layer have
  // different contracts; a filled pixel must never become an observation.
  const auto certified = layer + "_certified";
  const auto source = observed.exists(certified) ? certified : layer;
  gridMap.add(certified);
  auto& upper = gridMap.get(certified);
  upper.setConstant(std::numeric_limits<float>::quiet_NaN());
  // Include every source vertex of every bilinear patch adjacent to this
  // output vertex. Maximum height and unknown propagation remain conservative
  // when downsampling, upsampling, or using a shifted circular buffer.
  const double support = gridMap.getResolution() + observed.getResolution();
  for (grid_map::GridMapIterator cell(gridMap); !cell.isPastEnd(); ++cell) {
    grid_map::Position position;
    gridMap.getPosition(*cell, position);
    if (!observed.isInside((position.array() + support).matrix()) ||
        !observed.isInside((position.array() - support).matrix())) continue;
    bool success = false;
    const grid_map::SubmapGeometry region(observed, position,
                                         grid_map::Length::Constant(2 * support), success);
    if (!success) continue;
    float maximum = -std::numeric_limits<float>::infinity();
    bool known = true;
    for (grid_map::SubmapIterator vertex(region); !vertex.isPastEnd(); ++vertex) {
      const float height = observed.at(source, *vertex);
      if (!std::isfinite(height)) { known = false; break; }
      maximum = std::max(maximum, height);
    }
    if (known && std::isfinite(maximum)) upper((*cell)(0), (*cell)(1)) = maximum;
  }
}

void GridMapPreprocessing::denoise(grid_map::GridMap& gridMap, const std::string& layer) const {
  int kernelSize = std::max(1, std::min(parameters_.kernelSize, 5));  // must be 1, 3 or 5 for current image type, see doc of cv::medianBlur
  grid_map::smoothing::median(gridMap, layer, layer, kernelSize, 0, parameters_.numberOfRepeats);
}

void GridMapPreprocessing::changeResolution(grid_map::GridMap& gridMap, const std::string& layer) const {
  bool hasSameResolution = std::abs(gridMap.getResolution() - parameters_.resolution) < 1e-6;

  if (parameters_.resolution > 0.0 && !hasSameResolution) {
    grid_map::inpainting::resample(gridMap, layer, parameters_.resolution);
  }
}

void GridMapPreprocessing::inpaint(grid_map::GridMap& gridMap, const std::string& layer) const {
  grid_map::Matrix& map = gridMap.get(layer);
  bool needsInpainting = false;
  for (int col = 0; col < map.cols(); ++col) {
    for (int row = 0; row < map.rows(); ++row) {
      if (!std::isfinite(map(col, row))) {
        needsInpainting = true;
        break;
      }
    }
    if (needsInpainting) break;
  }
  if (needsInpainting) {
    grid_map::inpainting::minValues(gridMap, layer, layer); // In-place if supported
  }
}

bool containsFiniteValue(const grid_map::Matrix& map) {
  
  for (int col = 0; col < map.cols(); ++col) {
    for (int row = 0; row < map.rows(); ++row) {
      if (std::isfinite(map(col, row))) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace convex_plane_decomposition
