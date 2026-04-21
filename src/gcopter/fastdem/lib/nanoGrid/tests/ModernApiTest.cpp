/*
 * ModernApiTest.cpp
 *
 * Tests for modern C++17 API: Cell iteration, region/circle/neighbors,
 * Cell-based accessors, subRegion.
 */

#include "nanogrid/GridMap.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <set>

namespace nanogrid {

// ============================================================
// Cell iteration: for (auto cell : map)
// ============================================================

TEST(CellIteration, CountMatchesSize) {
  GridMap map;
  map.setGeometry(Length(5.0, 3.0), 0.5);
  map.add("layer");

  int count = 0;
  for (auto cell : map) {
    (void)cell;
    ++count;
  }
  EXPECT_EQ(map.getSize()(0) * map.getSize()(1), count);
}

TEST(CellIteration, LinearIndexCoversAllCells) {
  GridMap map;
  map.setGeometry(Length(2.0, 3.0), 0.5);
  map.add("layer");

  const int total = map.getSize()(0) * map.getSize()(1);
  std::set<Eigen::Index> indices;
  for (auto cell : map) {
    indices.insert(cell.index);
  }
  EXPECT_EQ(total, static_cast<int>(indices.size()));
  EXPECT_EQ(0, *indices.begin());
  EXPECT_EQ(total - 1, *indices.rbegin());
}

TEST(CellIteration, LogicalCoordsWithCircularBuffer) {
  GridMap map;
  map.setGeometry(Length(4.0, 3.0), 1.0);
  map.add("layer", 0.0);
  // Trigger circular buffer offset
  map.move(Position(-1.0, -1.0));
  EXPECT_FALSE(map.isDefaultStartIndex());

  std::set<std::pair<int, int>> logicalCoords;
  const int rows = map.getSize()(0);
  const int cols = map.getSize()(1);

  for (auto cell : map) {
    EXPECT_GE(cell.row, 0);
    EXPECT_LT(cell.row, rows);
    EXPECT_GE(cell.col, 0);
    EXPECT_LT(cell.col, cols);
    logicalCoords.insert({cell.row, cell.col});
  }
  // Every logical (row, col) pair should appear exactly once
  EXPECT_EQ(rows * cols, static_cast<int>(logicalCoords.size()));
}

TEST(CellIteration, CellsAndBeginEndEquivalent) {
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 0.5);
  map.add("layer");

  auto range = map.cells();
  auto it1 = range.begin();
  auto it2 = map.begin();
  int count = 0;
  while (it1 != range.end() && it2 != map.end()) {
    auto c1 = *it1;
    auto c2 = *it2;
    EXPECT_EQ(c1.index, c2.index);
    EXPECT_EQ(c1.row, c2.row);
    EXPECT_EQ(c1.col, c2.col);
    ++it1;
    ++it2;
    ++count;
  }
  EXPECT_EQ(map.getSize()(0) * map.getSize()(1), count);
}

// ============================================================
// Cell-based accessors
// ============================================================

TEST(CellAccessors, AtReadWrite) {
  GridMap map;
  map.setGeometry(Length(3.0, 3.0), 1.0);
  map.add("layer", 0.0f);

  for (auto cell : map) {
    map.at("layer", cell) = static_cast<float>(cell.row * 10 + cell.col);
  }
  for (auto cell : map) {
    float expected = static_cast<float>(cell.row * 10 + cell.col);
    EXPECT_FLOAT_EQ(expected, map.at("layer", cell));
  }
}

TEST(CellAccessors, PositionFromCell) {
  GridMap map;
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.add("layer");

  for (auto cell : map) {
    auto posFromCell = map.position(cell);
    auto posFromIndex = map.position(Index(cell.bufRow, cell.bufCol));
    ASSERT_TRUE(posFromCell.has_value());
    ASSERT_TRUE(posFromIndex.has_value());
    EXPECT_DOUBLE_EQ(posFromIndex->x(), posFromCell->x());
    EXPECT_DOUBLE_EQ(posFromIndex->y(), posFromCell->y());
  }
}

TEST(CellAccessors, PositionFromCellWithCircularBuffer) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0);
  map.add("layer", 0.0);
  map.move(Position(1.0, 1.0));

  for (auto cell : map) {
    auto pos = map.position(cell);
    ASSERT_TRUE(pos.has_value());
    // Position should be inside the map
    EXPECT_TRUE(map.isInside(*pos));
  }
}

TEST(CellAccessors, IsValidCell) {
  GridMap map;
  map.setGeometry(Length(3.0, 3.0), 1.0);
  map.add("layer", NAN);

  // All NaN → nothing valid
  for (auto cell : map) {
    EXPECT_FALSE(map.isValid(cell));
    EXPECT_FALSE(map.isValid(cell, "layer"));
  }

  // Set one cell valid
  auto it = map.begin();
  auto firstCell = *it;
  map.at("layer", firstCell) = 1.0f;
  EXPECT_TRUE(map.isValid(firstCell));
  EXPECT_TRUE(map.isValid(firstCell, "layer"));
}

// ============================================================
// region(center, size) — rectangular iteration
// ============================================================

TEST(Region, BasicCount) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);
  map.add("layer");

  // 5.0m region at 1.0m resolution: half-open interval discretization
  // yields 6 cells per axis (6x6 = 36)
  int count = 0;
  for (auto cell : map.region(Position(0.0, 0.0), Length(5.0, 5.0))) {
    (void)cell;
    ++count;
  }
  EXPECT_EQ(36, count);
}

TEST(Region, FullMapRegion) {
  GridMap map;
  map.setGeometry(Length(4.0, 6.0), 1.0);
  map.add("layer");

  int regionCount = 0;
  for (auto cell : map.region(map.getPosition(), map.getLength())) {
    (void)cell;
    ++regionCount;
  }
  EXPECT_EQ(map.getSize()(0) * map.getSize()(1), regionCount);
}

TEST(Region, ClampsToBoundary) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0, Position(0.0, 0.0));
  map.add("layer");

  // Request extends beyond map
  int count = 0;
  for (auto cell : map.region(Position(1.5, 1.5), Length(4.0, 4.0))) {
    (void)cell;
    ++count;
    // Every cell should be inside the map
    auto pos = map.position(cell);
    ASSERT_TRUE(pos.has_value());
    EXPECT_TRUE(map.isInside(*pos));
  }
  EXPECT_GT(count, 0);
  EXPECT_LT(count, map.getSize()(0) * map.getSize()(1));
}

TEST(Region, CompletelyOutsideIsEmpty) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0, Position(0.0, 0.0));
  map.add("layer");

  int count = 0;
  for (auto cell : map.region(Position(100.0, 100.0), Length(1.0, 1.0))) {
    (void)cell;
    ++count;
  }
  EXPECT_EQ(0, count);
}

TEST(Region, WithCircularBuffer) {
  GridMap map;
  map.setGeometry(Length(6.0, 6.0), 1.0);
  map.add("layer", 0.0);
  map.move(Position(2.0, 2.0));
  EXPECT_FALSE(map.isDefaultStartIndex());

  // Region at map center should still return valid cells
  int count = 0;
  for (auto cell : map.region(map.getPosition(), Length(4.0, 4.0))) {
    auto pos = map.position(cell);
    ASSERT_TRUE(pos.has_value());
    EXPECT_TRUE(map.isInside(*pos));
    ++count;
  }
  EXPECT_GT(count, 0);
}

// ============================================================
// circle(center, radius)
// ============================================================

TEST(Circle, BasicShape) {
  GridMap map;
  map.setGeometry(Length(20.0, 20.0), 1.0);
  map.add("layer");

  int count = 0;
  const double radius = 3.0;
  for (auto cell : map.circle(Position(0.0, 0.0), radius)) {
    auto pos = map.position(cell);
    ASSERT_TRUE(pos.has_value());
    // Cell center should be within radius (+ half diagonal tolerance)
    double dist = pos->norm();
    EXPECT_LE(dist, radius + map.getResolution());
    ++count;
  }
  // Circle area ~ pi * r^2 = ~28.3, expect roughly that many cells
  EXPECT_GT(count, 20);
  EXPECT_LT(count, 40);
}

TEST(Circle, ClampsToBoundary) {
  GridMap map;
  map.setGeometry(Length(6.0, 6.0), 1.0, Position(0.0, 0.0));
  map.add("layer");

  // Circle center at edge, radius extends outside
  int count = 0;
  for (auto cell : map.circle(Position(2.0, 2.0), 3.0)) {
    auto pos = map.position(cell);
    ASSERT_TRUE(pos.has_value());
    EXPECT_TRUE(map.isInside(*pos));
    ++count;
  }
  EXPECT_GT(count, 0);
}

TEST(Circle, CompletelyOutsideIsEmpty) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0);
  map.add("layer");

  int count = 0;
  for (auto cell : map.circle(Position(50.0, 50.0), 1.0)) {
    (void)cell;
    ++count;
  }
  EXPECT_EQ(0, count);
}

TEST(Circle, ZeroRadiusSingleCell) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);
  map.add("layer");

  int count = 0;
  for (auto cell : map.circle(Position(0.0, 0.0), 0.0)) {
    (void)cell;
    ++count;
  }
  // Zero radius should yield at most 1 cell (the center)
  EXPECT_LE(count, 1);
}

// ============================================================
// kernel() — precomputed neighborhood
// ============================================================

TEST(Kernel, CircularSymmetry) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);

  auto k = map.kernel(2.0);
  // Check symmetry: for every (dr, dc), (-dr, -dc) should exist
  std::set<std::pair<int, int>> offsets;
  for (const auto& e : k.entries) {
    offsets.insert({e.dr, e.dc});
  }
  for (const auto& e : k.entries) {
    EXPECT_TRUE(offsets.count({-e.dr, -e.dc}) > 0)
        << "Missing symmetric entry for (" << e.dr << ", " << e.dc << ")";
  }
}

TEST(Kernel, ContainsCenter) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);

  auto k = map.kernel(1.5);
  bool hasCenter = false;
  for (const auto& e : k.entries) {
    if (e.dr == 0 && e.dc == 0) {
      hasCenter = true;
      EXPECT_FLOAT_EQ(0.0f, e.dist_sq);
    }
  }
  EXPECT_TRUE(hasCenter);
}

TEST(Kernel, DistSqCorrect) {
  GridMap map;
  const double res = 0.5;
  map.setGeometry(Length(10.0, 10.0), res);

  auto k = map.kernel(1.5);
  for (const auto& e : k.entries) {
    float expected = static_cast<float>(res * res * (e.dr * e.dr + e.dc * e.dc));
    EXPECT_FLOAT_EQ(expected, e.dist_sq);
  }
}

TEST(Kernel, RectangularWindow) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);

  auto k = map.kernel(Size(3, 5));
  // 3x5 window → should have 3*5 = 15 entries
  EXPECT_EQ(15, static_cast<int>(k.entries.size()));
  EXPECT_EQ(-1, k.minDr);
  EXPECT_EQ(1, k.maxDr);
  EXPECT_EQ(-2, k.minDc);
  EXPECT_EQ(2, k.maxDc);
}

// ============================================================
// neighbors(cell, kernel)
// ============================================================

TEST(Neighbors, InteriorCellGetsAllEntries) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);
  map.add("layer", 1.0f);

  auto k = map.kernel(1.5);
  int expectedEntries = static_cast<int>(k.entries.size());

  // Pick a cell well inside the map
  GridMap::Cell interiorCell{};
  for (auto cell : map) {
    if (cell.row == 5 && cell.col == 5) {
      interiorCell = cell;
      break;
    }
  }

  int count = 0;
  for (auto n : map.neighbors(interiorCell, k)) {
    (void)n;
    ++count;
  }
  EXPECT_EQ(expectedEntries, count);
}

TEST(Neighbors, CornerCellSkipsBoundary) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);
  map.add("layer", 1.0f);

  auto k = map.kernel(1.5);
  int fullEntries = static_cast<int>(k.entries.size());

  // Pick corner cell (0, 0)
  GridMap::Cell cornerCell{};
  for (auto cell : map) {
    if (cell.row == 0 && cell.col == 0) {
      cornerCell = cell;
      break;
    }
  }

  int count = 0;
  for (auto n : map.neighbors(cornerCell, k)) {
    // Neighbor should be within valid bounds
    EXPECT_GE(n.row, 0);
    EXPECT_LT(n.row, map.getSize()(0));
    EXPECT_GE(n.col, 0);
    EXPECT_LT(n.col, map.getSize()(1));
    ++count;
  }
  EXPECT_LT(count, fullEntries);
  EXPECT_GT(count, 0);
}

TEST(Neighbors, EdgeCellPartialNeighbors) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);
  map.add("layer", 1.0f);

  auto k = map.kernel(1.5);
  int fullEntries = static_cast<int>(k.entries.size());

  // Pick edge cell (0, 5) — top row, middle column
  GridMap::Cell edgeCell{};
  for (auto cell : map) {
    if (cell.row == 0 && cell.col == 5) {
      edgeCell = cell;
      break;
    }
  }

  int count = 0;
  for (auto n : map.neighbors(edgeCell, k)) {
    (void)n;
    ++count;
  }
  // Should have fewer than full, but more than corner
  EXPECT_LT(count, fullEntries);
  EXPECT_GT(count, 0);
}

TEST(Neighbors, DistSqMatchesKernel) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);
  map.add("layer", 1.0f);

  auto k = map.kernel(2.0);

  // Interior cell
  GridMap::Cell center{};
  for (auto cell : map) {
    if (cell.row == 5 && cell.col == 5) {
      center = cell;
      break;
    }
  }

  int idx = 0;
  for (auto n : map.neighbors(center, k)) {
    EXPECT_FLOAT_EQ(k.entries[idx].dist_sq, n.dist_sq);
    ++idx;
  }
}

// ============================================================
// subRegion()
// ============================================================

TEST(SubRegion, BasicSuccess) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0);
  map.add("layer");

  // 4.0m at 1.0m resolution → 5 cells per axis (inclusive discretization)
  auto sr = map.subRegion(Position(0.0, 0.0), Length(4.0, 4.0));
  ASSERT_TRUE(sr.has_value());
  EXPECT_EQ(5, sr->size(0));
  EXPECT_EQ(5, sr->size(1));
}

TEST(SubRegion, CompletelyOutsideReturnsNullopt) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0);
  map.add("layer");

  auto sr = map.subRegion(Position(100.0, 100.0), Length(1.0, 1.0));
  EXPECT_FALSE(sr.has_value());
}

TEST(SubRegion, ClampedToMap) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0, Position(0.0, 0.0));
  map.add("layer");

  // Request larger than map, centered at map center
  auto sr = map.subRegion(Position(0.0, 0.0), Length(10.0, 10.0));
  ASSERT_TRUE(sr.has_value());
  EXPECT_EQ(map.getSize()(0), sr->size(0));
  EXPECT_EQ(map.getSize()(1), sr->size(1));
}

// ============================================================
// Modern value-returning API
// ============================================================

TEST(ModernApi, IndexAndPositionRoundTrip) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 0.5, Position(1.0, 2.0));
  map.add("layer");

  Position queryPos(2.0, 3.0);
  auto idx = map.index(queryPos);
  ASSERT_TRUE(idx.has_value());

  auto recoveredPos = map.position(*idx);
  ASSERT_TRUE(recoveredPos.has_value());

  // Should round-trip to within half a cell
  EXPECT_NEAR(queryPos.x(), recoveredPos->x(), map.getResolution());
  EXPECT_NEAR(queryPos.y(), recoveredPos->y(), map.getResolution());
}

TEST(ModernApi, IndexOutsideReturnsNullopt) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0);
  map.add("layer");

  EXPECT_FALSE(map.index(Position(100.0, 100.0)).has_value());
}

TEST(ModernApi, GetByPosition) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0, Position(0.0, 0.0));
  map.add("elevation", 42.0f);

  auto val = map.get("elevation", Position(0.0, 0.0));
  ASSERT_TRUE(val.has_value());
  EXPECT_FLOAT_EQ(42.0f, *val);
}

TEST(ModernApi, GetNanReturnsNullopt) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0);
  map.add("layer");  // default NAN

  auto val = map.get("layer", Position(0.0, 0.0));
  EXPECT_FALSE(val.has_value());
}

TEST(ModernApi, GetByIndex) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0);
  map.add("layer", 7.0f);

  auto val = map.get("layer", Index(0, 0));
  ASSERT_TRUE(val.has_value());
  EXPECT_FLOAT_EQ(7.0f, *val);
}

TEST(ModernApi, GetOutsidePositionReturnsNullopt) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0);
  map.add("layer", 1.0f);

  EXPECT_FALSE(map.get("layer", Position(100.0, 100.0)).has_value());
}

TEST(ModernApi, Position3) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0, Position(0.0, 0.0));
  map.add("elevation", 5.0f);

  auto p3 = map.position3("elevation", Index(0, 0));
  ASSERT_TRUE(p3.has_value());
  EXPECT_DOUBLE_EQ(5.0, p3->z());
}

TEST(ModernApi, Position3NanReturnsNullopt) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0);
  map.add("elevation");  // NAN

  EXPECT_FALSE(map.position3("elevation", Index(0, 0)).has_value());
}

TEST(ModernApi, Vector3) {
  GridMap map;
  map.setGeometry(Length(4.0, 4.0), 1.0);
  map.add("normal_x", 0.0f);
  map.add("normal_y", 0.0f);
  map.add("normal_z", 1.0f);

  auto v = map.vector3("normal_", Index(0, 0));
  ASSERT_TRUE(v.has_value());
  EXPECT_DOUBLE_EQ(0.0, v->x());
  EXPECT_DOUBLE_EQ(0.0, v->y());
  EXPECT_DOUBLE_EQ(1.0, v->z());
}

TEST(ModernApi, SubmapBasic) {
  GridMap map;
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  map.add("layer", 1.0f);

  // 4.0m at 1.0m resolution → 5 cells per axis (inclusive discretization)
  auto sub = map.submap(Position(0.0, 0.0), Length(4.0, 4.0));
  ASSERT_TRUE(sub.has_value());
  EXPECT_EQ(5, sub->getSize()(0));
  EXPECT_EQ(5, sub->getSize()(1));
  // Data should be copied
  EXPECT_FLOAT_EQ(1.0f, (*sub)["layer"](0, 0));
}

}  // namespace nanogrid
