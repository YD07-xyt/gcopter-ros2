/*
 * GridMap.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "nanogrid/GridMap.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>

#include "nanogrid/detail/GridMapMath.hpp"

using std::isfinite;

namespace nanogrid {

GridMap::GridMap(const std::vector<std::string>& layers) {
  position_.setZero();
  length_.setZero();
  resolution_ = 0.0;
  size_.setZero();
  startIndex_.setZero();
  timestamp_ = 0;
  layers_ = layers;

  for (auto& layer : layers_) {
    data_.try_emplace(layer);
  }
}

GridMap::GridMap() : GridMap(std::vector<std::string>()) {}

void GridMap::setGeometry(const Length& length, const double resolution,
                          const Position& position) {
  assert(length(0) > 0.0);
  assert(length(1) > 0.0);
  assert(resolution > 0.0);

  Size size;
  size(0) = static_cast<int>(
      round(length(0) / resolution));  // There is no round() function in Eigen.
  size(1) = static_cast<int>(round(length(1) / resolution));
  resize(size);
  clearAll();

  resolution_ = resolution;
  length_ = (size_.cast<double>() * resolution_).matrix();
  position_ = position;
  startIndex_.setZero();
}

bool GridMap::hasSameLayers(const GridMap& other) const {
  return std::all_of(
      layers_.begin(), layers_.end(),
      [&](const std::string& layer) { return other.exists(layer); });
}

void GridMap::add(const std::string& layer, const float value) {
  add(layer, Matrix::Constant(size_(0), size_(1), value));
}

void GridMap::add(const std::string& layer, const Matrix& data) {
  assert(size_(0) == data.rows());
  assert(size_(1) == data.cols());

  if (exists(layer)) {
    // Type exists already, overwrite its data.
    data_.at(layer) = data;
  } else {
    // Type does not exist yet, add type and data.
    data_.try_emplace(layer, data);
    layers_.push_back(layer);
  }
}

bool GridMap::exists(const std::string& layer) const {
  return data_.count(layer) > 0;
}

const Matrix& GridMap::get(const std::string& layer) const {
  try {
    return data_.at(layer);
  } catch (const std::out_of_range&) {
    throw std::out_of_range("GridMap::get(...) : No map layer '" + layer +
                            "' available.");
  }
}

Matrix& GridMap::get(const std::string& layer) {
  try {
    return data_.at(layer);
  } catch (const std::out_of_range&) {
    throw std::out_of_range("GridMap::get(...) : No map layer of type '" +
                            layer + "' available.");
  }
}

const Matrix& GridMap::operator[](const std::string& layer) const {
  return get(layer);
}

Matrix& GridMap::operator[](const std::string& layer) { return get(layer); }

bool GridMap::erase(const std::string& layer) {
  const auto dataIterator = data_.find(layer);
  if (dataIterator == data_.end()) {
    return false;
  }
  data_.erase(dataIterator);

  const auto layerIterator = std::find(layers_.begin(), layers_.end(), layer);
  if (layerIterator == layers_.end()) {
    return false;
  }
  layers_.erase(layerIterator);
  return true;
}

const std::vector<std::string>& GridMap::getLayers() const { return layers_; }

float& GridMap::at(const std::string& layer, const Index& index) {
  try {
    return data_.at(layer)(index(0), index(1));
  } catch (const std::out_of_range&) {
    throw std::out_of_range("GridMap::at(...) : No map layer '" + layer +
                            "' available.");
  }
}

float GridMap::at(const std::string& layer, const Index& index) const {
  try {
    return data_.at(layer)(index(0), index(1));
  } catch (const std::out_of_range&) {
    throw std::out_of_range("GridMap::at(...) : No map layer '" + layer +
                            "' available.");
  }
}

bool GridMap::getIndex(const Position& position, Index& index) const {
  return getIndexFromPosition(index, position, length_, position_, resolution_,
                              size_, startIndex_);
}

bool GridMap::getPosition(const Index& index, Position& position) const {
  return getPositionFromIndex(position, index, length_, position_, resolution_,
                              size_, startIndex_);
}

// ============================================================
// Modern C++17 API
// ============================================================

std::optional<Index> GridMap::index(const Position& position) const {
  Index idx;
  if (getIndex(position, idx)) {
    return idx;
  }
  return std::nullopt;
}

std::optional<Position> GridMap::position(const Index& index) const {
  Position pos;
  if (getPosition(index, pos)) {
    return pos;
  }
  return std::nullopt;
}

std::optional<Position3> GridMap::position3(const std::string& layer,
                                            const Index& index) const {
  const auto value = at(layer, index);
  if (!std::isfinite(value)) {
    return std::nullopt;
  }
  Position pos2d;
  getPosition(index, pos2d);
  return Position3{pos2d.x(), pos2d.y(), static_cast<double>(value)};
}

std::optional<Vector3> GridMap::vector3(const std::string& layerPrefix,
                                        const Index& index) const {
  Vector3 vec{at(layerPrefix + "x", index), at(layerPrefix + "y", index),
              at(layerPrefix + "z", index)};
  if (!std::isfinite(vec[0]) || !std::isfinite(vec[1]) ||
      !std::isfinite(vec[2])) {
    return std::nullopt;
  }
  return vec;
}

std::optional<float> GridMap::get(const std::string& layer,
                                  const Position& position) const {
  auto idx = index(position);
  if (!idx) {
    return std::nullopt;
  }
  return get(layer, *idx);
}

std::optional<float> GridMap::get(const std::string& layer,
                                  const Index& index) const {
  const auto val = at(layer, index);
  if (!std::isfinite(val)) {
    return std::nullopt;
  }
  return val;
}

std::optional<GridMap> GridMap::submap(const Position& position,
                                       const Length& length) const {
  bool isSuccess;
  auto result = getSubmap(position, length, isSuccess);
  if (isSuccess) {
    return result;
  }
  return std::nullopt;
}

// ============================================================

bool GridMap::isInside(const Position& position) const {
  return checkIfPositionWithinMap(position, length_, position_);
}

bool GridMap::isValid(DataType value) const { return isfinite(value); }

bool GridMap::isValid(const Index& index) const {
  return std::any_of(
      layers_.begin(), layers_.end(),
      [&](const std::string& layer) { return isValid(at(layer, index)); });
}

bool GridMap::isValid(const Index& index, const std::string& layer) const {
  return isValid(at(layer, index));
}

bool GridMap::isValid(const Index& index,
                      const std::vector<std::string>& layers) const {
  if (layers.empty()) {
    return false;
  }
  return std::all_of(
      layers.begin(), layers.end(),
      [&](const std::string& layer) { return isValid(index, layer); });
}

std::optional<GridMap::SubRegion> GridMap::subRegion(
    const Position& position, const Length& length) const {
  Index startIdx;
  Size sz;
  Position pos;
  Length len;
  Index reqIdx;
  if (!getSubmapInformation(startIdx, sz, pos, len, reqIdx,
                            position, length, length_, position_,
                            resolution_, size_, startIndex_)) {
    return std::nullopt;
  }
  return SubRegion{startIdx, sz};
}

GridMap GridMap::getSubmap(const Position& position, const Length& length,
                           bool& isSuccess) const {
  Index index;
  return getSubmap(position, length, index, isSuccess);
}

GridMap GridMap::getSubmap(const Position& position, const Length& length,
                           Index& /*indexInSubmap*/, bool& isSuccess) const {
  // Submap to generate.
  GridMap submap(layers_);
  submap.setTimestamp(timestamp_);
  submap.setFrameId(frameId_);

  // Get submap geometric information.
  Index submapStartIndex;
  Size submapSize;
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;
  isSuccess = getSubmapInformation(submapStartIndex, submapSize, submapPosition,
                                   submapLength, requestedIndexInSubmap,
                                   position, length, length_, position_,
                                   resolution_, size_, startIndex_);
  if (!isSuccess) {
    return GridMap{layers_};
  }
  submap.setGeometry(submapLength, resolution_, submapPosition);
  submap.startIndex_.setZero();  // Because of the way we copy the data below.

  // Copy data.
  std::vector<BufferRegion> bufferRegions;

  if (!getBufferRegionsForSubmap(bufferRegions, submapStartIndex,
                                 submap.getSize(), size_, startIndex_)) {
    isSuccess = false;
    return GridMap{layers_};
  }

  for (const auto& data : data_) {
    for (const auto& bufferRegion : bufferRegions) {
      Index index = bufferRegion.getStartIndex();
      Size size = bufferRegion.getSize();

      if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::TopLeft) {
        submap.data_[data.first].topLeftCorner(size(0), size(1)) =
            data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() ==
                 BufferRegion::Quadrant::TopRight) {
        submap.data_[data.first].topRightCorner(size(0), size(1)) =
            data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() ==
                 BufferRegion::Quadrant::BottomLeft) {
        submap.data_[data.first].bottomLeftCorner(size(0), size(1)) =
            data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() ==
                 BufferRegion::Quadrant::BottomRight) {
        submap.data_[data.first].bottomRightCorner(size(0), size(1)) =
            data.second.block(index(0), index(1), size(0), size(1));
      }
    }
  }

  isSuccess = true;
  return submap;
}

GridMap GridMap::getTransformedMap(const Eigen::Isometry3d& transform,
                                   const std::string& heightLayerName,
                                   const std::string& newFrameId,
                                   const double sampleRatio) const {
  // Check if height layer is valid.
  if (!exists(heightLayerName)) {
    throw std::out_of_range("GridMap::getTransformedMap(...) : No map layer '" +
                            heightLayerName + "' available.");
  }

  // Initialization.
  std::vector<Position3> positionSamples;
  Position3 center;
  Index newIndex;

  const double sampleLength = resolution_ * sampleRatio;

  // Find edges in new coordinate frame.
  const double halfLengthX = length_.x() * 0.5;
  const double halfLengthY = length_.y() * 0.5;
  const Position3 topLeftCorner(position_.x() + halfLengthX,
                                position_.y() + halfLengthY, 0.0);
  const Position3 topRightCorner(position_.x() + halfLengthX,
                                 position_.y() - halfLengthY, 0.0);
  const Position3 bottomLeftCorner(position_.x() - halfLengthX,
                                   position_.y() + halfLengthY, 0.0);
  const Position3 bottomRightCorner(position_.x() - halfLengthX,
                                    position_.y() - halfLengthY, 0.0);

  std::vector<Position3> newEdges;
  newEdges.reserve(4);
  newEdges.push_back(transform * topLeftCorner);
  newEdges.push_back(transform * topRightCorner);
  newEdges.push_back(transform * bottomLeftCorner);
  newEdges.push_back(transform * bottomRightCorner);

  // Find new grid center.
  Position3 newCenter = Position3::Zero();
  for (const auto& newEdge : newEdges) {
    newCenter += newEdge;
  }
  newCenter *= 0.25;

  // Find new grid length.
  Length maxLengthFromCenter = Length(0.0, 0.0);
  for (const auto& newEdge : newEdges) {
    Position3 positionCenterToEdge = newEdge - newCenter;
    maxLengthFromCenter.x() =
        std::fmax(std::fabs(positionCenterToEdge.x()), maxLengthFromCenter.x());
    maxLengthFromCenter.y() =
        std::fmax(std::fabs(positionCenterToEdge.y()), maxLengthFromCenter.y());
  }
  Length newLength = 2.0 * maxLengthFromCenter;

  // Create new grid map.
  GridMap newMap(layers_);
  newMap.setTimestamp(timestamp_);
  newMap.setFrameId(newFrameId);
  newMap.setGeometry(newLength, resolution_,
                     Position(newCenter.x(), newCenter.y()));
  newMap.startIndex_.setZero();

  for (auto cell : *this) {
    // Get position at current index.
    auto pos3 = position3(heightLayerName, Index(cell.bufRow, cell.bufCol));
    if (!pos3) {
      continue;
    }
    center = *pos3;

    // Sample four points around the center cell.
    positionSamples.clear();

    if (sampleRatio > 0.0) {
      positionSamples.reserve(5);
      positionSamples.push_back(center);
      positionSamples.emplace_back(center.x() - sampleLength, center.y(),
                                   center.z());
      positionSamples.emplace_back(center.x() + sampleLength, center.y(),
                                   center.z());
      positionSamples.emplace_back(center.x(), center.y() - sampleLength,
                                   center.z());
      positionSamples.emplace_back(center.x(), center.y() + sampleLength,
                                   center.z());
    } else {
      positionSamples.push_back(center);
    }

    // Transform the sampled points and register to the new map.
    for (const auto& position : positionSamples) {
      const Position3 transformedPosition = transform * position;

      // Get new index.
      auto newIdxOpt = newMap.index(
              Position(transformedPosition.x(), transformedPosition.y()));
      if (!newIdxOpt) {
        continue;
      }
      newIndex = *newIdxOpt;

      // Check if we have already assigned a value (preferably larger height
      // values -> inpainting).
      const auto newExistingValue = newMap.at(heightLayerName, newIndex);
      if (!std::isnan(newExistingValue) &&
          newExistingValue > transformedPosition.z()) {
        continue;
      }

      // Copy the layers.
      for (const auto& layer : layers_) {
        const auto currentValueInOldGrid = data_.at(layer)(cell.index);
        auto& newValue = newMap.at(layer, newIndex);
        if (layer == heightLayerName) {
          newValue = static_cast<float>(transformedPosition.z());
        }  // adjust height
        else {
          newValue = currentValueInOldGrid;
        }  // re-assign
      }
    }
  }

  return newMap;
}

void GridMap::setPosition(const Position& position) { position_ = position; }

bool GridMap::move(const Position& position,
                   std::vector<BufferRegion>& newRegions) {
  Index indexShift;
  Position positionShift = position - position_;
  getIndexShiftFromPositionShift(indexShift, positionShift, resolution_);
  Position alignedPositionShift;
  getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution_);

  // Delete fields that fall out of map (and become empty cells).
  for (int i = 0; i < indexShift.size(); i++) {
    if (indexShift(i) != 0) {
      if (abs(indexShift(i)) >= getSize()(i)) {
        // Entire map is dropped.
        clearAll();
        newRegions.emplace_back(Index(0, 0), getSize(),
                                BufferRegion::Quadrant::Undefined);
      } else {
        // Drop cells out of map.
        int sign = (indexShift(i) > 0 ? 1 : -1);
        int startIndex = startIndex_(i) - (sign < 0 ? 1 : 0);
        int endIndex = startIndex - sign + indexShift(i);
        int nCells = abs(indexShift(i));
        int index = (sign > 0 ? startIndex : endIndex);
        wrapIndexToRange(index, getSize()(i));

        if (index + nCells <= getSize()(i)) {
          // One region to drop.
          if (i == 0) {
            clearRows(index, nCells);
            newRegions.emplace_back(Index(index, 0), Size(nCells, getSize()(1)),
                                    BufferRegion::Quadrant::Undefined);
          } else if (i == 1) {
            clearCols(index, nCells);
            newRegions.emplace_back(Index(0, index), Size(getSize()(0), nCells),
                                    BufferRegion::Quadrant::Undefined);
          }
        } else {
          // Two regions to drop.
          int firstIndex = index;
          int firstNCells = getSize()(i) - firstIndex;
          if (i == 0) {
            clearRows(firstIndex, firstNCells);
            newRegions.emplace_back(Index(firstIndex, 0),
                                    Size(firstNCells, getSize()(1)),
                                    BufferRegion::Quadrant::Undefined);
          } else if (i == 1) {
            clearCols(firstIndex, firstNCells);
            newRegions.emplace_back(Index(0, firstIndex),
                                    Size(getSize()(0), firstNCells),
                                    BufferRegion::Quadrant::Undefined);
          }

          int secondIndex = 0;
          int secondNCells = nCells - firstNCells;
          if (i == 0) {
            clearRows(secondIndex, secondNCells);
            newRegions.emplace_back(Index(secondIndex, 0),
                                    Size(secondNCells, getSize()(1)),
                                    BufferRegion::Quadrant::Undefined);
          } else if (i == 1) {
            clearCols(secondIndex, secondNCells);
            newRegions.emplace_back(Index(0, secondIndex),
                                    Size(getSize()(0), secondNCells),
                                    BufferRegion::Quadrant::Undefined);
          }
        }
      }
    }
  }

  // Update information.
  startIndex_ += indexShift;
  wrapIndexToRange(startIndex_, getSize());
  position_ += alignedPositionShift;

  // Check if map has been moved at all.
  return indexShift.any();
}

bool GridMap::move(const Position& position) {
  std::vector<BufferRegion> newRegions;
  return move(position, newRegions);
}

bool GridMap::addDataFrom(const GridMap& other, bool extendMap,
                          bool overwriteData, bool copyAllLayers,
                          std::vector<std::string> layers) {
  // Set the layers to copy.
  if (copyAllLayers) {
    layers = other.getLayers();
  }

  // Resize map.
  if (extendMap) {
    extendToInclude(other);
  }

  // Check if all layers to copy exist and add missing layers.
  for (const auto& layer : layers) {
    if (std::find(layers_.begin(), layers_.end(), layer) == layers_.end()) {
      add(layer);
    }
  }
  // Copy data.
  for (auto cell : *this) {
    auto pos = position(cell);
    if (!pos || !other.isInside(*pos)) {
      continue;
    }
    auto otherIdx = other.index(*pos);
    if (!otherIdx) {
      continue;
    }
    for (const auto& layer : layers) {
      if (!other.isValid(*otherIdx, layer)) {
        continue;
      }
      if (!overwriteData && std::isfinite(data_.at(layer)(cell.index))) {
        continue;
      }
      data_.at(layer)(cell.index) = other.at(layer, *otherIdx);
    }
  }

  return true;
}

bool GridMap::extendToInclude(const GridMap& other) {
  // Get dimension of maps.
  Position topLeftCorner(position_.x() + length_.x() / 2.0,
                         position_.y() + length_.y() / 2.0);
  Position bottomRightCorner(position_.x() - length_.x() / 2.0,
                             position_.y() - length_.y() / 2.0);
  Position topLeftCornerOther(
      other.getPosition().x() + other.getLength().x() / 2.0,
      other.getPosition().y() + other.getLength().y() / 2.0);
  Position bottomRightCornerOther(
      other.getPosition().x() - other.getLength().x() / 2.0,
      other.getPosition().y() - other.getLength().y() / 2.0);
  // Check if map needs to be resized.
  bool resizeMap = false;
  Position extendedMapPosition = position_;
  Length extendedMapLength = length_;
  if (topLeftCornerOther.x() > topLeftCorner.x()) {
    extendedMapPosition.x() +=
        (topLeftCornerOther.x() - topLeftCorner.x()) / 2.0;
    extendedMapLength.x() += topLeftCornerOther.x() - topLeftCorner.x();
    resizeMap = true;
  }
  if (topLeftCornerOther.y() > topLeftCorner.y()) {
    extendedMapPosition.y() +=
        (topLeftCornerOther.y() - topLeftCorner.y()) / 2.0;
    extendedMapLength.y() += topLeftCornerOther.y() - topLeftCorner.y();
    resizeMap = true;
  }
  if (bottomRightCornerOther.x() < bottomRightCorner.x()) {
    extendedMapPosition.x() -=
        (bottomRightCorner.x() - bottomRightCornerOther.x()) / 2.0;
    extendedMapLength.x() += bottomRightCorner.x() - bottomRightCornerOther.x();
    resizeMap = true;
  }
  if (bottomRightCornerOther.y() < bottomRightCorner.y()) {
    extendedMapPosition.y() -=
        (bottomRightCorner.y() - bottomRightCornerOther.y()) / 2.0;
    extendedMapLength.y() += bottomRightCorner.y() - bottomRightCornerOther.y();
    resizeMap = true;
  }
  // Resize map and copy data to new map.
  if (resizeMap) {
    GridMap mapCopy = *this;
    setGeometry(extendedMapLength, resolution_, extendedMapPosition);
    // Align new map with old one.
    Vector shift = position_ - mapCopy.getPosition();
    shift.x() = std::fmod(shift.x(), resolution_);
    shift.y() = std::fmod(shift.y(), resolution_);
    if (std::abs(shift.x()) < resolution_ / 2.0) {
      position_.x() -= shift.x();
    } else {
      position_.x() += resolution_ - shift.x();
    }
    if (size_.x() % 2 != mapCopy.getSize().x() % 2) {
      position_.x() += -std::copysign(resolution_ / 2.0, shift.x());
    }
    if (std::abs(shift.y()) < resolution_ / 2.0) {
      position_.y() -= shift.y();
    } else {
      position_.y() += resolution_ - shift.y();
    }
    if (size_.y() % 2 != mapCopy.getSize().y() % 2) {
      position_.y() += -std::copysign(resolution_ / 2.0, shift.y());
    }
    // Copy data.
    for (auto cell : *this) {
      if (isValid(cell)) {
        continue;
      }
      auto pos = position(cell);
      if (!pos || !mapCopy.isInside(*pos)) {
        continue;
      }
      auto copyIdx = mapCopy.index(*pos);
      if (!copyIdx) {
        continue;
      }
      for (const auto& layer : layers_) {
        data_.at(layer)(cell.index) = mapCopy.at(layer, *copyIdx);
      }
    }
  }
  return true;
}

void GridMap::setTimestamp(const Time timestamp) { timestamp_ = timestamp; }

Time GridMap::getTimestamp() const { return timestamp_; }

void GridMap::resetTimestamp() { timestamp_ = 0; }

void GridMap::setFrameId(const std::string& frameId) { frameId_ = frameId; }

const std::string& GridMap::getFrameId() const { return frameId_; }

const Length& GridMap::getLength() const { return length_; }

const Position& GridMap::getPosition() const { return position_; }

double GridMap::getResolution() const { return resolution_; }

const Size& GridMap::getSize() const { return size_; }

void GridMap::setStartIndex(const Index& startIndex) {
  startIndex_ = startIndex;
}

const Index& GridMap::getStartIndex() const { return startIndex_; }

bool GridMap::isDefaultStartIndex() const { return (startIndex_ == 0).all(); }

void GridMap::convertToDefaultStartIndex() {
  if (isDefaultStartIndex()) {
    return;
  }

  std::vector<BufferRegion> bufferRegions;
  if (!getBufferRegionsForSubmap(bufferRegions, startIndex_, size_, size_,
                                 startIndex_)) {
    throw std::out_of_range("Cannot access submap of this size.");
  }

  for (auto& data : data_) {
    auto tempData(data.second);
    for (const auto& bufferRegion : bufferRegions) {
      Index index = bufferRegion.getStartIndex();
      Size size = bufferRegion.getSize();

      if (bufferRegion.getQuadrant() == BufferRegion::Quadrant::TopLeft) {
        tempData.topLeftCorner(size(0), size(1)) =
            data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() ==
                 BufferRegion::Quadrant::TopRight) {
        tempData.topRightCorner(size(0), size(1)) =
            data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() ==
                 BufferRegion::Quadrant::BottomLeft) {
        tempData.bottomLeftCorner(size(0), size(1)) =
            data.second.block(index(0), index(1), size(0), size(1));
      } else if (bufferRegion.getQuadrant() ==
                 BufferRegion::Quadrant::BottomRight) {
        tempData.bottomRightCorner(size(0), size(1)) =
            data.second.block(index(0), index(1), size(0), size(1));
      }
    }
    data.second = tempData;
  }

  startIndex_.setZero();
}

Position GridMap::getClosestPositionInMap(const Position& position) const {
  if (getSize().x() < 1 || getSize().y() < 1) {
    return position_;
  }

  if (isInside(position)) {
    return position;
  }

  Position positionInMap = position;

  // Find edges.
  const double halfLengthX = length_.x() * 0.5;
  const double halfLengthY = length_.y() * 0.5;
  const Position3 topLeftCorner(position_.x() + halfLengthX,
                                position_.y() + halfLengthY, 0.0);
  const Position3 topRightCorner(position_.x() + halfLengthX,
                                 position_.y() - halfLengthY, 0.0);
  const Position3 bottomLeftCorner(position_.x() - halfLengthX,
                                   position_.y() + halfLengthY, 0.0);
  const Position3 bottomRightCorner(position_.x() - halfLengthX,
                                    position_.y() - halfLengthY, 0.0);

  // Find constraints.
  const double maxX = topRightCorner.x();
  const double minX = bottomRightCorner.x();
  const double maxY = bottomLeftCorner.y();
  const double minY = bottomRightCorner.y();

  // Clip to box constraints and correct for indexing precision.
  // Points on the border can lead to invalid indices because the cells
  // represent half open intervals, i.e. [...).
  positionInMap.x() = std::fmin(positionInMap.x(),
                                maxX - std::numeric_limits<double>::epsilon());
  positionInMap.y() = std::fmin(positionInMap.y(),
                                maxY - std::numeric_limits<double>::epsilon());

  positionInMap.x() = std::fmax(positionInMap.x(),
                                minX + std::numeric_limits<double>::epsilon());
  positionInMap.y() = std::fmax(positionInMap.y(),
                                minY + std::numeric_limits<double>::epsilon());

  return positionInMap;
}

void GridMap::clear(const std::string& layer) {
  try {
    data_.at(layer).setConstant(NAN);
  } catch (const std::out_of_range&) {
    throw std::out_of_range("GridMap::clear(...) : No map layer '" + layer +
                            "' available.");
  }
}

void GridMap::clearAll() {
  for (auto& data : data_) {
    data.second.setConstant(NAN);
  }
}

void GridMap::clearRows(unsigned int index, unsigned int nRows) {
  for (auto& layer : layers_) {
    data_.at(layer).block(index, 0, nRows, getSize()(1)).setConstant(NAN);
  }
}

void GridMap::clearCols(unsigned int index, unsigned int nCols) {
  for (auto& layer : layers_) {
    data_.at(layer).block(0, index, getSize()(0), nCols).setConstant(NAN);
  }
}

void GridMap::resize(const Index& size) {
  size_ = size;
  for (auto& data : data_) {
    data.second.resize(size_(0), size_(1));
  }
}

// ============================================================
// Cell-based accessors
// ============================================================

float& GridMap::at(const std::string& layer, const Cell& cell) {
  return data_.at(layer)(cell.index);
}

float GridMap::at(const std::string& layer, const Cell& cell) const {
  return data_.at(layer)(cell.index);
}

std::optional<Position> GridMap::position(const Cell& cell) const {
  return position(Index(cell.bufRow, cell.bufCol));
}

bool GridMap::isValid(const Cell& cell) const {
  return std::any_of(
      layers_.begin(), layers_.end(),
      [&](const std::string& layer) {
        return isValid(data_.at(layer)(cell.index));
      });
}

bool GridMap::isValid(const Cell& cell, const std::string& layer) const {
  return isValid(data_.at(layer)(cell.index));
}

// ============================================================
// Spatial iteration API
// ============================================================

GridMap::RegionRange GridMap::region(const Position& center,
                                    const Length& size) const {
  const int rows = size_(0);
  const int cols = size_(1);

  // Compute the two corners in world coordinates
  Position topLeft(center.x() + size.x() / 2.0,
                   center.y() + size.y() / 2.0);
  Position bottomRight(center.x() - size.x() / 2.0,
                       center.y() - size.y() / 2.0);

  // Check if region has any overlap with map
  const Position mapMin = position_.array() - length_ / 2.0;
  const Position mapMax = position_.array() + length_ / 2.0;
  if (bottomRight.x() > mapMax.x() || topLeft.x() < mapMin.x() ||
      bottomRight.y() > mapMax.y() || topLeft.y() < mapMin.y()) {
    return {0, 0, 0, 0, rows, cols, startIndex_(0), startIndex_(1)};
  }

  // Clamp to map boundaries
  boundPositionToRange(topLeft, length_, position_);
  boundPositionToRange(bottomRight, length_, position_);

  // Convert to physical buffer indices
  Index tlPhys, brPhys;
  if (!getIndexFromPosition(tlPhys, topLeft, length_, position_, resolution_,
                            size_, startIndex_) ||
      !getIndexFromPosition(brPhys, bottomRight, length_, position_,
                            resolution_, size_, startIndex_)) {
    return {0, 0, 0, 0, rows, cols, startIndex_(0), startIndex_(1)};
  }

  // Physical → logical
  Index tlLog = getIndexFromBufferIndex(tlPhys, size_, startIndex_);
  Index brLog = getIndexFromBufferIndex(brPhys, size_, startIndex_);

  int startRow = std::min(tlLog(0), brLog(0));
  int endRow = std::max(tlLog(0), brLog(0)) + 1;
  int startCol = std::min(tlLog(1), brLog(1));
  int endCol = std::max(tlLog(1), brLog(1)) + 1;

  // Clamp to valid range
  startRow = std::max(0, startRow);
  endRow = std::min(rows, endRow);
  startCol = std::max(0, startCol);
  endCol = std::min(cols, endCol);

  return {startRow, endRow, startCol, endCol, rows, cols, startIndex_(0),
          startIndex_(1)};
}

GridMap::CircleRange GridMap::circle(const Position& center,
                                    double radius) const {
  Length boundingSize(2.0 * radius, 2.0 * radius);
  RegionRange boundingRegion = region(center, boundingSize);

  double originX = position_.x() + 0.5 * length_.x();
  double originY = position_.y() + 0.5 * length_.y();
  double centerRow = (originX - center.x()) / resolution_ - 0.5;
  double centerCol = (originY - center.y()) / resolution_ - 0.5;

  return {boundingRegion, centerRow, centerCol, radius * radius, resolution_};
}

GridMap::Kernel GridMap::kernel(double radius) const {
  Kernel nbr;
  const int rCells = static_cast<int>(std::ceil(radius / resolution_));
  const double radiusSq = radius * radius;
  const double res = resolution_;

  nbr.minDr = rCells;
  nbr.maxDr = -rCells;
  nbr.minDc = rCells;
  nbr.maxDc = -rCells;

  for (int dr = -rCells; dr <= rCells; ++dr) {
    for (int dc = -rCells; dc <= rCells; ++dc) {
      double dsq = res * res * (dr * dr + dc * dc);
      if (dsq <= radiusSq) {
        nbr.entries.push_back({dr, dc, static_cast<float>(dsq)});
        nbr.minDr = std::min(nbr.minDr, dr);
        nbr.maxDr = std::max(nbr.maxDr, dr);
        nbr.minDc = std::min(nbr.minDc, dc);
        nbr.maxDc = std::max(nbr.maxDc, dc);
      }
    }
  }
  return nbr;
}

GridMap::Kernel GridMap::kernel(const Size& window) const {
  Kernel nbr;
  const int halfRow = window(0) / 2;
  const int halfCol = window(1) / 2;

  nbr.minDr = -halfRow;
  nbr.maxDr = halfRow;
  nbr.minDc = -halfCol;
  nbr.maxDc = halfCol;

  const float res = static_cast<float>(resolution_);
  for (int dr = -halfRow; dr <= halfRow; ++dr) {
    for (int dc = -halfCol; dc <= halfCol; ++dc) {
      float dsq = res * res * static_cast<float>(dr * dr + dc * dc);
      nbr.entries.push_back({dr, dc, dsq});
    }
  }
  return nbr;
}

GridMap::NeighborRange GridMap::neighbors(const Cell& cell,
                                         const Kernel& nbr) const {
  const Kernel::Entry* data =
      nbr.entries.empty() ? nullptr : nbr.entries.data();
  const Kernel::Entry* dataEnd = data + nbr.entries.size();

  bool allValid = (cell.row + nbr.minDr >= 0) &&
                  (cell.row + nbr.maxDr < size_(0)) &&
                  (cell.col + nbr.minDc >= 0) &&
                  (cell.col + nbr.maxDc < size_(1));

  return {data,
          dataEnd,
          cell.row,
          cell.col,
          size_(0),
          size_(1),
          startIndex_(0),
          startIndex_(1),
          allValid};
}

}  // namespace nanogrid
