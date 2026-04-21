/*
 * GridMapTest.cpp
 *
 *  Created on: Aug 26, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "nanogrid/GridMap.hpp"

// gtest
#include <gtest/gtest.h>

namespace nanogrid {

TEST(GridMap, CopyConstructor) {
  GridMap map({"layer_a", "layer_b"});
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.1, 0.2));
  map["layer_a"].setConstant(1.0);
  map["layer_b"].setConstant(2.0);
  GridMap mapCopy(map);
  EXPECT_EQ(map.getSize()[0], mapCopy.getSize()[0]);
  EXPECT_EQ(map.getSize()[1], mapCopy.getSize()[1]);
  EXPECT_EQ(map.getLength().x(), mapCopy.getLength().x());
  EXPECT_EQ(map.getLength().y(), mapCopy.getLength().y());
  EXPECT_EQ(map.getPosition().x(), mapCopy.getPosition().x());
  EXPECT_EQ(map.getPosition().y(), mapCopy.getPosition().y());
  EXPECT_EQ(map.getLayers().size(), mapCopy.getLayers().size());
  EXPECT_EQ(map["layer_a"](0, 0), mapCopy["layer_a"](0, 0));
  EXPECT_EQ(map["layer_b"](0, 0), mapCopy["layer_b"](0, 0));
}

TEST(GridMap, CopyAssign)
{
  GridMap map({"layer_a", "layer_b"});
  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.1, 0.2));
  map["layer_a"].setConstant(1.0);
  map["layer_b"].setConstant(2.0);
  GridMap mapCopy;
  mapCopy = map;
  EXPECT_EQ(map.getSize()[0], mapCopy.getSize()[0]);
  EXPECT_EQ(map.getSize()[1], mapCopy.getSize()[1]);
  EXPECT_EQ(map.getLength().x(), mapCopy.getLength().x());
  EXPECT_EQ(map.getLength().y(), mapCopy.getLength().y());
  EXPECT_EQ(map.getPosition().x(), mapCopy.getPosition().x());
  EXPECT_EQ(map.getPosition().y(), mapCopy.getPosition().y());
  EXPECT_EQ(map.getLayers().size(), mapCopy.getLayers().size());
  EXPECT_EQ(map["layer_a"](0, 0), mapCopy["layer_a"](0, 0));
  EXPECT_EQ(map["layer_b"](0, 0), mapCopy["layer_b"](0, 0));
}

TEST(GridMap, Move)
{
  GridMap map;
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(8, 5)
  map.add("layer", 0.0);
  map.move(Position(-3.0, -2.0));
  Index startIndex = map.getStartIndex();

  EXPECT_EQ(3, startIndex(0));
  EXPECT_EQ(2, startIndex(1));
  
  Eigen::Matrix<bool, 8, 5> isValidExpected;
  isValidExpected << false, false, false, false, false, // clang-format off
                     false, false, false, false, false,
                     false, false, false, false, false,
                     false, false, true,  true,  true,
                     false, false, true,  true,  true,
                     false, false, true,  true,  true,
                     false, false, true,  true,  true,
                     false, false, true,  true,  true; // clang-format on
  for(int row{0}; row < 8; row++){
    for(int col{0}; col < 5; col++){
      EXPECT_EQ(map.isValid(Index(row, col)), isValidExpected(row, col)) << "Value of map.isValid at ["<<row << ", " << col <<"] is unexpected!";
    }
  }

}

TEST(GridMap, Transform)
{
  // Initial map.
  GridMap map;
  constexpr auto heightLayerName = "height";

  map.setGeometry(Length(1.0, 2.0), 0.1, Position(0.0, 0.0));
  map.add(heightLayerName, 0.0);
  map.get(heightLayerName)(0,0) = 1.0;

  // Transformation (90° rotation).
  Eigen::Isometry3d transform;

  transform.translation().x() = 0.0;
  transform.translation().y() = 0.0;
  transform.translation().z() = 0.0;

  transform.linear()(0,0) =  0.0;
  transform.linear()(0,1) = -1.0;
  transform.linear()(0,2) =  0.0;

  transform.linear()(1,0) =  1.0;
  transform.linear()(1,1) =  0.0;
  transform.linear()(1,2) =  0.0;

  transform.linear()(2,0) =  0.0;
  transform.linear()(2,1) =  0.0;
  transform.linear()(2,2) =  1.0;

  // Apply affine transformation.
  const GridMap transformedMap = map.getTransformedMap(transform, heightLayerName, map.getFrameId(), 0.25);

  // Check if map has been rotated by 90° about z
  EXPECT_NEAR(map.getLength().x(), transformedMap.getLength().y(), 1e-6);
  EXPECT_NEAR(map.getLength().y(), transformedMap.getLength().x(), 1e-6);
  EXPECT_EQ(map.get(heightLayerName).size(), transformedMap.get(heightLayerName).size());
  EXPECT_DOUBLE_EQ(map.get(heightLayerName)(0,0), transformedMap.get(heightLayerName)(19,0));
}

TEST(GridMap, ClipToMap)
{
  GridMap map({"layer_a", "layer_b"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.5, 0.5));
  map["layer_a"].setConstant(1.0);
  map["layer_b"].setConstant(2.0);

  const Position positionInMap = Position(0.4, 0.3); // position located inside the map
  const Position positionOutMap = Position(1.0, 2.0); // position located outside the map

  const Position clippedPositionInMap = map.getClosestPositionInMap(positionInMap);
  const Position clippedPositionOutMap = map.getClosestPositionInMap(positionOutMap);

  // Check if position-in-map remains unchanged.
  EXPECT_NEAR(clippedPositionInMap.x(),positionInMap.x(), 1e-6);
  EXPECT_NEAR(clippedPositionInMap.y(), positionInMap.y(), 1e-6);

  // Check if position-out-map is indeed outside the map.
  EXPECT_TRUE(!map.isInside(positionOutMap));

  // Check if position-out-map has been projected into the map.
  EXPECT_TRUE(map.isInside(clippedPositionOutMap));
}



TEST(GridMap, ClipToMap2)
{
  GridMap map({"types"});
  map.setGeometry(Length(1.0, 1.0), 0.05, Position(0.0, 0.0));

  // Test 8 points outside of map.
  /*
   * A  B  C
   *  +---+
   *  |   |         X
   * D|   |E        ^
   *  |   |         |
   *  +---+     Y<--+
   * F  G  H
   *
   * Note: Position to index alignment is a half open interval.
   *       An example position of 0.5 is assigned to the upper index.
   *       The interval in the current example is: 
   *       Position: [...)[0.485 ... 0.5)[0.5 ... 0.505)[...)
   *       Index:      8          9           10          11
   */

  Index insideIndex;
  Position outsidePosition;

  // Point A
  outsidePosition = Position(1.0, 1.0);
  auto closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  auto insideIdxOpt = map.index(closestInsidePosition);
  bool isInside = insideIdxOpt.has_value();
  if (isInside) insideIndex = *insideIdxOpt;

  auto expectedPosition = Position(0.5, 0.5);
  auto expectedIndex = Index(0, 0);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point B
  outsidePosition = Position(1.0, 0.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  insideIdxOpt = map.index(closestInsidePosition);
  isInside = insideIdxOpt.has_value();
  if (isInside) insideIndex = *insideIdxOpt;

  expectedPosition = Position(0.5, 0.0);
  expectedIndex = Index(0, 10);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point C
  outsidePosition = Position(1.0, -1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  insideIdxOpt = map.index(closestInsidePosition);
  isInside = insideIdxOpt.has_value();
  if (isInside) insideIndex = *insideIdxOpt;

  expectedPosition = Position(0.5, -0.5);
  expectedIndex = Index(0, 19);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point D
  outsidePosition = Position(0.0, 1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  insideIdxOpt = map.index(closestInsidePosition);
  isInside = insideIdxOpt.has_value();
  if (isInside) insideIndex = *insideIdxOpt;

  expectedPosition = Position(0.0, 0.5);
  expectedIndex = Index(10, 0);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point E
  outsidePosition = Position(0.0, -1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  insideIdxOpt = map.index(closestInsidePosition);
  isInside = insideIdxOpt.has_value();
  if (isInside) insideIndex = *insideIdxOpt;

  expectedPosition = Position(0.0, -0.5);
  expectedIndex = Index(10, 19);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point F
  outsidePosition = Position(-1.0, 1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  insideIdxOpt = map.index(closestInsidePosition);
  isInside = insideIdxOpt.has_value();
  if (isInside) insideIndex = *insideIdxOpt;

  expectedPosition = Position(-0.5, 0.5);
  expectedIndex = Index(19, 0);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point G
  outsidePosition = Position(-1.0, 0.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  insideIdxOpt = map.index(closestInsidePosition);
  isInside = insideIdxOpt.has_value();
  if (isInside) insideIndex = *insideIdxOpt;

  expectedPosition = Position(-0.5, 0.0);
  expectedIndex = Index(19, 10);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;

  // Point H
  outsidePosition = Position(-1.0, -1.0);
  closestInsidePosition = map.getClosestPositionInMap(outsidePosition);
  insideIdxOpt = map.index(closestInsidePosition);
  isInside = insideIdxOpt.has_value();
  if (isInside) insideIndex = *insideIdxOpt;

  expectedPosition = Position(-0.5, -0.5);
  expectedIndex = Index(19, 19);

  // Check position.
  EXPECT_DOUBLE_EQ(expectedPosition.x(), closestInsidePosition.x());
  EXPECT_DOUBLE_EQ(expectedPosition.y(), closestInsidePosition.y());
  
  // Check index.
  EXPECT_EQ(expectedIndex.x(), insideIndex.x()) << "closestInsidePosition" << closestInsidePosition;
  EXPECT_EQ(expectedIndex.y(), insideIndex.y()) << "closestInsidePosition" << closestInsidePosition;
  
  // Check if index is inside.
  EXPECT_TRUE(isInside) << "position is: " << std::endl
                        << closestInsidePosition << std::endl 
                        << " index is: " << std::endl
                        << insideIndex << std::endl;
}

TEST(AddDataFrom, ExtendMapAligned)
{
  GridMap map1;
  GridMap map2;
  map1.setGeometry(Length(5.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(5, 5)
  map1.add("zero", 0.0f);
  map1.add("one", 1.0f);

  map2.setGeometry(Length(3.1, 3.1), 1.0, Position(2.0, 2.0));
  map2.add("one", 1.1f);
  map2.add("two", 2.0f);

  map1.addDataFrom(map2, true, true, true);

  EXPECT_TRUE(map1.exists("two"));
  EXPECT_TRUE(map1.isInside(Position(3.0, 3.0)));
  EXPECT_DOUBLE_EQ(6.0, map1.getLength().x());
  EXPECT_DOUBLE_EQ(6.0, map1.getLength().y());
  EXPECT_DOUBLE_EQ(0.5, map1.getPosition().x());
  EXPECT_DOUBLE_EQ(0.5, map1.getPosition().y());
  EXPECT_NEAR(1.1, *map1.get("one", Position(2, 2)), 1e-4);
  EXPECT_DOUBLE_EQ(1.0, *map1.get("one", Position(-2, -2)));
  EXPECT_DOUBLE_EQ(0.0, *map1.get("zero", Position(0.0, 0.0)));
}

TEST(AddDataFrom, ExtendMapNotAligned)
{
  GridMap map1;
  GridMap map2;
  map1.setGeometry(Length(6.1, 6.1), 1.0, Position(0.0, 0.0)); // bufferSize(6, 6)
  map1.add("nan");
  map1.add("one", 1.0);
  map1.add("zero", 0.0);

  map2.setGeometry(Length(3.1, 3.1), 1.0, Position(3.2, 3.2));
  map2.add("nan", 1.0f);
  map2.add("one", 1.1f);
  map2.add("two", 2.0f);

  std::vector<std::string> stringVector;
  stringVector.emplace_back("nan");
  map1.addDataFrom(map2, true, false, false, stringVector);
  auto idx = map1.index(Position(-2, -2));
  ASSERT_TRUE(idx.has_value());

  EXPECT_FALSE(map1.exists("two"));
  EXPECT_TRUE(map1.isInside(Position(4.0, 4.0)));
  EXPECT_DOUBLE_EQ(8.0, map1.getLength().x());
  EXPECT_DOUBLE_EQ(8.0, map1.getLength().y());
  EXPECT_DOUBLE_EQ(1.0, map1.getPosition().x());
  EXPECT_DOUBLE_EQ(1.0, map1.getPosition().y());
  EXPECT_FALSE(map1.isValid(*idx, "nan"));
  EXPECT_DOUBLE_EQ(1.0, *map1.get("one", Position(0.0, 0.0)));
  EXPECT_DOUBLE_EQ(1.0, *map1.get("nan", Position(3.0, 3.0)));
}

TEST(AddDataFrom, CopyData)
{
  GridMap map1;
  GridMap map2;
  map1.setGeometry(Length(5.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(5, 5)
  map1.add("zero", 0.0);
  map1.add("one");

  map2.setGeometry(Length(3.1, 3.1), 1.0, Position(2.0, 2.0));
  map2.add("one", 1.0);
  map2.add("two", 2.0);

  map1.addDataFrom(map2, false, false, true);
  auto idx = map1.index(Position(-2, -2));
  ASSERT_TRUE(idx.has_value());

  EXPECT_TRUE(map1.exists("two"));
  EXPECT_FALSE(map1.isInside(Position(3.0, 3.0)));
  EXPECT_DOUBLE_EQ(5.0, map1.getLength().x());
  EXPECT_DOUBLE_EQ(5.0, map1.getLength().y());
  EXPECT_DOUBLE_EQ(0.0, map1.getPosition().x());
  EXPECT_DOUBLE_EQ(0.0, map1.getPosition().y());
  EXPECT_DOUBLE_EQ(1.0, *map1.get("one", Position(2, 2)));
  EXPECT_FALSE(map1.isValid(*idx, "one"));
  EXPECT_DOUBLE_EQ(0.0, *map1.get("zero", Position(0.0, 0.0)));
}

// ============================================================
// Modern C++17 API Tests
// ============================================================

TEST(ModernAPI, Index) {
  GridMap map({"layer"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.0, 0.0));

  // Position inside map
  auto idx = map.index(Position(0.0, 0.0));
  ASSERT_TRUE(idx.has_value());
  EXPECT_EQ(5, (*idx)(0));
  EXPECT_EQ(5, (*idx)(1));

  // Position outside map
  auto none = map.index(Position(10.0, 10.0));
  EXPECT_FALSE(none.has_value());
}

TEST(ModernAPI, Position) {
  GridMap map({"layer"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.0, 0.0));

  // Valid index
  auto pos = map.position(Index(0, 0));
  ASSERT_TRUE(pos.has_value());
  EXPECT_NEAR(0.45, pos->x(), 1e-6);
  EXPECT_NEAR(0.45, pos->y(), 1e-6);

  // Out-of-range index
  auto none = map.position(Index(100, 100));
  EXPECT_FALSE(none.has_value());
}

TEST(ModernAPI, Position3) {
  GridMap map({"height"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.0, 0.0));
  map.at("height", Index(5, 5)) = 3.14f;

  // Valid cell with finite value
  auto pos3 = map.position3("height", Index(5, 5));
  ASSERT_TRUE(pos3.has_value());
  EXPECT_NEAR(3.14, pos3->z(), 1e-5);

  // Cell with NaN value (default)
  auto none = map.position3("height", Index(0, 0));
  EXPECT_FALSE(none.has_value());
}

TEST(ModernAPI, Vector3) {
  GridMap map({"normal_x", "normal_y", "normal_z"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.0, 0.0));
  map.at("normal_x", Index(5, 5)) = 0.0f;
  map.at("normal_y", Index(5, 5)) = 0.0f;
  map.at("normal_z", Index(5, 5)) = 1.0f;

  // Valid vector
  auto vec = map.vector3("normal_", Index(5, 5));
  ASSERT_TRUE(vec.has_value());
  EXPECT_DOUBLE_EQ(0.0, vec->x());
  EXPECT_DOUBLE_EQ(0.0, vec->y());
  EXPECT_DOUBLE_EQ(1.0, vec->z());

  // NaN components → nullopt
  auto none = map.vector3("normal_", Index(0, 0));
  EXPECT_FALSE(none.has_value());
}

TEST(ModernAPI, Submap) {
  GridMap map({"layer"});
  map.setGeometry(Length(5.0, 5.0), 0.1, Position(0.0, 0.0));
  map["layer"].setConstant(1.0f);

  // Valid submap
  auto sub = map.submap(Position(0.0, 0.0), Length(2.0, 2.0));
  ASSERT_TRUE(sub.has_value());
  EXPECT_GE(sub->getSize()(0), 20);
  EXPECT_GE(sub->getSize()(1), 20);
  EXPECT_LE(sub->getSize()(0), 21);  // discretization may add 1
  EXPECT_LE(sub->getSize()(1), 21);
  EXPECT_TRUE(sub->exists("layer"));

  // Submap completely outside → nullopt
  auto none = map.submap(Position(100.0, 100.0), Length(1.0, 1.0));
  EXPECT_FALSE(none.has_value());
}

TEST(ModernAPI, GetByIndex) {
  GridMap map({"elevation"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.0, 0.0));

  // Valid cell
  map.at("elevation", Index(5, 5)) = 3.14f;
  auto val = map.get("elevation", Index(5, 5));
  ASSERT_TRUE(val.has_value());
  EXPECT_FLOAT_EQ(*val, 3.14f);

  // NaN cell (default)
  auto nanVal = map.get("elevation", Index(0, 0));
  EXPECT_FALSE(nanVal.has_value());
}

TEST(ModernAPI, GetByPosition) {
  GridMap map({"elevation"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.0, 0.0));
  map.at("elevation", Index(5, 5)) = 2.71f;

  // Valid position with valid data
  auto pos = map.position(Index(5, 5));
  ASSERT_TRUE(pos.has_value());
  auto val = map.get("elevation", *pos);
  ASSERT_TRUE(val.has_value());
  EXPECT_FLOAT_EQ(*val, 2.71f);

  // Position outside map
  auto outside = map.get("elevation", Position(10.0, 10.0));
  EXPECT_FALSE(outside.has_value());

  // Position inside map but NaN cell
  Position originPos(0.0, 0.0);
  auto nanVal = map.get("elevation", originPos);
  // Origin maps to center cell which is NaN by default
  // (unless it happens to be Index(5,5))
  if (auto idx = map.index(originPos)) {
    if ((*idx)(0) != 5 || (*idx)(1) != 5) {
      EXPECT_FALSE(nanVal.has_value());
    }
  }
}

TEST(ModernAPI, GetConsistencyWithAt) {
  GridMap map({"layer"});
  map.setGeometry(Length(2.0, 2.0), 0.1, Position(0.0, 0.0));

  // Set some cells to known values
  map.at("layer", Index(3, 7)) = 42.0f;
  map.at("layer", Index(8, 2)) = -1.5f;

  // get() should return same as at() for valid cells
  auto v1 = map.get("layer", Index(3, 7));
  ASSERT_TRUE(v1.has_value());
  EXPECT_FLOAT_EQ(*v1, map.at("layer", Index(3, 7)));

  auto v2 = map.get("layer", Index(8, 2));
  ASSERT_TRUE(v2.has_value());
  EXPECT_FLOAT_EQ(*v2, map.at("layer", Index(8, 2)));
}

TEST(ModernAPI, Cells) {
  GridMap map({"layer"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.0, 0.0));
  auto& data = map["layer"];
  data.setConstant(0.0f);

  // Write via cells()
  for (auto cell : map) {
    data(cell.index) = static_cast<float>(cell.index);
  }

  // Verify all cells were written
  size_t count = 0;
  for (auto cell : map) {
    EXPECT_FLOAT_EQ(data(cell.index), static_cast<float>(cell.index));
    ++count;
  }
  EXPECT_EQ(count, static_cast<size_t>(map.getSize().prod()));
}

TEST(ModernAPI, CellsConsistencyWithDirectLoop) {
  GridMap map({"layer"});
  map.setGeometry(Length(2.0, 2.0), 0.1, Position(0.0, 0.0));
  auto& data = map["layer"];

  // Fill with known pattern
  for (Eigen::Index i = 0; i < data.size(); ++i) {
    data(i) = static_cast<float>(i) * 0.5f;
  }

  // cells() should visit same indices in same order as direct loop
  Eigen::Index direct_i = 0;
  for (auto cell : map) {
    EXPECT_EQ(cell.index, direct_i);
    EXPECT_FLOAT_EQ(data(cell.index), static_cast<float>(direct_i) * 0.5f);
    ++direct_i;
  }
}

TEST(ModernAPI, CellsRowCol) {
  GridMap map({"layer"});
  map.setGeometry(Length(1.0, 2.0), 0.5, Position(0.0, 0.0));
  // Size: 2 rows x 4 cols

  const int rows = map.getSize()(0);
  size_t count = 0;
  for (auto cell : map) {
    // Default startIndex (0,0): logical == physical
    int expectedRow = static_cast<int>(cell.index % rows);
    int expectedCol = static_cast<int>(cell.index / rows);
    EXPECT_EQ(cell.row, expectedRow);
    EXPECT_EQ(cell.col, expectedCol);
    ++count;
  }
  EXPECT_EQ(count, static_cast<size_t>(map.getSize().prod()));
}

TEST(ModernAPI, CellsRowColAfterMove) {
  GridMap map({"layer"});
  map.setGeometry(Length(5.1, 5.1), 1.0, Position(0.0, 0.0));
  map["layer"].setConstant(0.0f);
  map.move(Position(-2.0, -1.0));

  const auto startIdx = map.getStartIndex();
  EXPECT_TRUE((startIdx != 0).any());  // Circular buffer active

  const int rows = map.getSize()(0);
  const int cols = map.getSize()(1);

  for (auto cell : map) {
    // Forward transform: logical -> physical (same as MapIndexer)
    int bufRow = cell.row + startIdx(0);
    if (bufRow >= rows) bufRow -= rows;
    int bufCol = cell.col + startIdx(1);
    if (bufCol >= cols) bufCol -= cols;

    int expectedPhysRow = static_cast<int>(cell.index % rows);
    int expectedPhysCol = static_cast<int>(cell.index / rows);
    EXPECT_EQ(bufRow, expectedPhysRow);
    EXPECT_EQ(bufCol, expectedPhysCol);
  }
}

TEST(ModernAPI, CellsLinearAccess) {
  GridMap map({"layer"});
  map.setGeometry(Length(1.0, 1.0), 0.1, Position(0.0, 0.0));
  auto& data = map["layer"];
  data.setConstant(0.0f);

  // cell.index provides linear access to Eigen matrix
  for (auto cell : map) {
    data(cell.index) = 1.0f;
  }

  for (Eigen::Index i = 0; i < data.size(); ++i) {
    EXPECT_FLOAT_EQ(data(i), 1.0f);
  }
}

// ============================================================
// Spatial iteration API tests
// ============================================================

TEST(SpatialIteration, RectBasic) {
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  map["layer"].setConstant(0.0f);

  size_t count = 0;
  for (auto cell : map.region(Position(0.0, 0.0), Length(4.0, 4.0))) {
    EXPECT_GE(cell.row, 0);
    EXPECT_LT(cell.row, map.getSize()(0));
    EXPECT_GE(cell.col, 0);
    EXPECT_LT(cell.col, map.getSize()(1));
    ++count;
  }
  EXPECT_GT(count, 0u);
  EXPECT_LE(count, static_cast<size_t>(map.getSize().prod()));
}

TEST(SpatialIteration, RectAfterMove) {
  GridMap map({"layer"});
  map.setGeometry(Length(10.1, 10.1), 1.0, Position(0.0, 0.0));
  map["layer"].setConstant(1.0f);
  map.move(Position(-3.0, -2.0));

  size_t count = 0;
  for (auto cell : map.region(Position(-3.0, -2.0), Length(4.0, 4.0))) {
    EXPECT_GE(cell.index, 0);
    EXPECT_LT(cell.index, static_cast<Eigen::Index>(map.getSize().prod()));
    map["layer"](cell.index) = 2.0f;
    ++count;
  }
  EXPECT_GT(count, 0u);
}

TEST(SpatialIteration, CircleBasic) {
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 0.5, Position(0.0, 0.0));

  double radius = 2.0;
  size_t count = 0;
  for (auto cell : map.circle(Position(0.0, 0.0), radius)) {
    // Verify cell index is valid
    EXPECT_GE(cell.index, 0);
    EXPECT_LT(cell.index, static_cast<Eigen::Index>(map.getSize().prod()));
    ++count;
  }
  // Circle area ~= pi * 4 = 12.56 m^2, cell area = 0.25 m^2 → ~50 cells
  EXPECT_GE(count, 30u);
  EXPECT_LE(count, 70u);
}

TEST(SpatialIteration, CircleAfterMove) {
  GridMap map({"layer"});
  map.setGeometry(Length(10.1, 10.1), 1.0, Position(0.0, 0.0));
  map["layer"].setConstant(0.0f);
  map.move(Position(-2.0, -2.0));

  size_t count = 0;
  for (auto cell : map.circle(Position(-2.0, -2.0), 2.0)) {
    EXPECT_GE(cell.index, 0);
    EXPECT_LT(cell.index, static_cast<Eigen::Index>(map.getSize().prod()));
    map["layer"](cell.index) = 1.0f;
    ++count;
  }
  EXPECT_GT(count, 0u);
}

TEST(SpatialIteration, NeighborsCircle) {
  GridMap map({"layer"});
  map.setGeometry(Length(10.1, 10.1), 1.0, Position(0.0, 0.0));
  map["layer"].setConstant(0.0f);

  auto reg = map.kernel(2.0);

  // Pick center cell (5, 5) — interior cell
  GridMap::Cell center{0, 5, 5, 5, 5};
  int physRow = (5 + map.getStartIndex()(0)) % map.getSize()(0);
  int physCol = (5 + map.getStartIndex()(1)) % map.getSize()(1);
  center.index =
      static_cast<Eigen::Index>(physCol) * map.getSize()(0) + physRow;

  double radius = 2.0;
  size_t count = 0;
  for (auto cell : map.neighbors(center, reg)) {
    double dr = (cell.row - center.row) * map.getResolution();
    double dc = (cell.col - center.col) * map.getResolution();
    EXPECT_LE(dr * dr + dc * dc, radius * radius + 1e-9);
    ++count;
  }
  EXPECT_GT(count, 0u);
}

TEST(SpatialIteration, NeighborsRect) {
  GridMap map({"layer"});
  map.setGeometry(Length(10.1, 10.1), 1.0, Position(0.0, 0.0));

  auto reg = map.kernel(Size(3, 3));

  // Pick center cell (5, 5) — interior cell
  GridMap::Cell center{0, 5, 5, 5, 5};
  int physRow = (5 + map.getStartIndex()(0)) % map.getSize()(0);
  int physCol = (5 + map.getStartIndex()(1)) % map.getSize()(1);
  center.index =
      static_cast<Eigen::Index>(physCol) * map.getSize()(0) + physRow;

  size_t count = 0;
  for (auto cell : map.neighbors(center, reg)) {
    EXPECT_GE(cell.row, center.row - 1);
    EXPECT_LE(cell.row, center.row + 1);
    EXPECT_GE(cell.col, center.col - 1);
    EXPECT_LE(cell.col, center.col + 1);
    ++count;
  }
  EXPECT_EQ(count, 9u);
}

TEST(SpatialIteration, RectOutside) {
  GridMap map({"layer"});
  map.setGeometry(Length(5.0, 5.0), 1.0, Position(0.0, 0.0));

  size_t count = 0;
  for (auto cell : map.region(Position(100.0, 100.0), Length(1.0, 1.0))) {
    ++count;
    (void)cell;
  }
  EXPECT_EQ(count, 0u);
}

TEST(SpatialIteration, NeighborsAtBorder) {
  GridMap map({"layer"});
  map.setGeometry(Length(5.1, 5.1), 1.0, Position(0.0, 0.0));

  auto reg = map.kernel(1.5);

  // Cell at corner (0, 0)
  GridMap::Cell corner{0, 0, 0, 0, 0};
  int physRow = (0 + map.getStartIndex()(0)) % map.getSize()(0);
  int physCol = (0 + map.getStartIndex()(1)) % map.getSize()(1);
  corner.index =
      static_cast<Eigen::Index>(physCol) * map.getSize()(0) + physRow;

  size_t count = 0;
  for (auto cell : map.neighbors(corner, reg)) {
    EXPECT_GE(cell.row, 0);
    EXPECT_GE(cell.col, 0);
    ++count;
  }
  // At corner, fewer neighbors than interior
  EXPECT_GT(count, 0u);
  EXPECT_LT(count, reg.entries.size());
}

// Helper: get world position from Cell using its linear index (column-major)
static std::optional<Position> positionFromCell(const GridMap& map,
                                                const GridMap::Cell& cell) {
  int rows = map.getSize()(0);
  int physRow = static_cast<int>(cell.index % rows);
  int physCol = static_cast<int>(cell.index / rows);
  return map.position(Index(physRow, physCol));
}

TEST(SpatialIteration, CircleOffMapCenter) {
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  map["layer"].setConstant(1.0f);

  // Center is outside the map, but circle (r=7) overlaps with it
  Position offCenter(8.0, 0.0);
  double radius = 7.0;

  size_t count = 0;
  for (auto cell : map.circle(offCenter, radius)) {
    // Every iterated cell must actually be within the circle
    auto pos = positionFromCell(map, cell);
    ASSERT_TRUE(pos.has_value());
    double dx = pos->x() - offCenter.x();
    double dy = pos->y() - offCenter.y();
    EXPECT_LE(dx * dx + dy * dy, radius * radius + 1e-6);
    ++count;
  }
  // The circle should cover some cells (the overlap region)
  EXPECT_GT(count, 0u);

  // Verify no false positives: count should be less than the full map
  size_t totalCells = static_cast<size_t>(map.getSize().prod());
  EXPECT_LT(count, totalCells);
}

TEST(SpatialIteration, CircleFullyOutside) {
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));

  // Circle entirely outside the map
  size_t count = 0;
  for (auto cell : map.circle(Position(20.0, 20.0), 3.0)) {
    ++count;
    (void)cell;
  }
  EXPECT_EQ(count, 0u);
}

TEST(SpatialIteration, CircleOffMapCenterAfterMove) {
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  map["layer"].setConstant(1.0f);

  // Move the map to activate circular buffer
  map.move(Position(3.0, 0.0));

  // Center outside the moved map, circle overlaps
  Position offCenter(12.0, 0.0);
  double radius = 7.0;

  size_t count = 0;
  for (auto cell : map.circle(offCenter, radius)) {
    auto pos = positionFromCell(map, cell);
    ASSERT_TRUE(pos.has_value());
    double dx = pos->x() - offCenter.x();
    double dy = pos->y() - offCenter.y();
    EXPECT_LE(dx * dx + dy * dy, radius * radius + 1e-6);
    ++count;
  }
  EXPECT_GT(count, 0u);
}

}  // namespace nanogrid