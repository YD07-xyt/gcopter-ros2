/*
 * benchmark_spatial.cpp
 *
 * Compares nanoGrid spatial iteration API vs grid_map legacy iterators.
 */

#include <nanogrid/nanogrid.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>

#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using Clock = std::chrono::high_resolution_clock;

struct BenchResult {
  std::string name;
  double ms;
};

double bench(std::function<void()> fn, int warmup = 2, int runs = 10) {
  for (int i = 0; i < warmup; ++i) fn();
  double total = 0.0;
  for (int i = 0; i < runs; ++i) {
    auto t0 = Clock::now();
    fn();
    auto t1 = Clock::now();
    total += std::chrono::duration<double, std::milli>(t1 - t0).count();
  }
  return total / runs;
}

void printResults(const std::string& title,
                  const std::vector<BenchResult>& results) {
  std::cout << "\n" << title << "\n" << std::string(60, '-') << "\n";
  std::cout << std::left << std::setw(40) << "Method" << std::right
            << std::setw(10) << "Time (ms)" << std::setw(10) << "Ratio"
            << "\n" << std::string(60, '-') << "\n";
  double fastest = results[0].ms;
  for (auto& r : results) fastest = std::min(fastest, r.ms);
  for (auto& r : results) {
    std::cout << std::left << std::setw(40) << r.name << std::right
              << std::setw(10) << std::fixed << std::setprecision(2) << r.ms
              << std::setw(9) << std::setprecision(1) << (r.ms / fastest)
              << "x\n";
  }
}

void bench_submap(int N) {
  const double resolution = 20.0 / N;
  const auto center = Eigen::Vector2d(0.0, 0.0);
  const auto submapLength = grid_map::Length(10.0, 10.0);

  grid_map::GridMap gm({"layer"});
  gm.setGeometry(grid_map::Length(20.0, 20.0), resolution);
  gm["layer"].setRandom();

  nanogrid::GridMap nm({"layer"});
  nm.setGeometry(nanogrid::Length(20.0, 20.0), resolution);
  nm["layer"].setRandom();

  std::vector<BenchResult> results;

  {
    bool isSuccess;
    auto submap = gm.getSubmap(center, submapLength, isSuccess);
    const auto submapStart = submap.getStartIndex();
    const auto submapBufSize = submap.getSize();
    results.push_back({"grid_map::SubmapIterator",
       bench([&]() {
         float sum = 0;
         const auto& data = gm["layer"];
         for (grid_map::SubmapIterator it(gm, submapStart, submapBufSize);
              !it.isPastEnd(); ++it) {
           sum += data((*it)(0), (*it)(1));
         }
         volatile float sink = sum; (void)sink;
       })});
  }

  results.push_back({"nanoGrid::rect()",
     bench([&]() {
       float sum = 0;
       for (auto cell : nm.region(nanogrid::Position(0.0, 0.0),
                                nanogrid::Length(10.0, 10.0))) {
         sum += nm["layer"](cell.index);
       }
       volatile float sink = sum; (void)sink;
     })});

  printResults("Rect/Submap (" + std::to_string(N) + "x" + std::to_string(N) + ")", results);
}

void bench_circle(int N) {
  const double resolution = 20.0 / N;
  const auto center = Eigen::Vector2d(0.0, 0.0);
  const double radius = 5.0;

  grid_map::GridMap gm({"layer"});
  gm.setGeometry(grid_map::Length(20.0, 20.0), resolution);
  gm["layer"].setRandom();

  nanogrid::GridMap nm({"layer"});
  nm.setGeometry(nanogrid::Length(20.0, 20.0), resolution);
  nm["layer"].setRandom();

  std::vector<BenchResult> results;

  results.push_back({"grid_map::CircleIterator",
     bench([&]() {
       float sum = 0;
       const auto& data = gm["layer"];
       for (grid_map::CircleIterator it(gm, center, radius);
            !it.isPastEnd(); ++it) {
         sum += data((*it)(0), (*it)(1));
       }
       volatile float sink = sum; (void)sink;
     })});

  results.push_back({"nanoGrid::circle()",
     bench([&]() {
       float sum = 0;
       for (auto cell : nm.circle(nanogrid::Position(0.0, 0.0), radius)) {
         sum += nm["layer"](cell.index);
       }
       volatile float sink = sum; (void)sink;
     })});

  printResults("Circle r=5m (" + std::to_string(N) + "x" + std::to_string(N) + ")", results);
}

void bench_neighbors(int N) {
  const double resolution = 20.0 / N;
  const double radius = 2.0 * resolution;

  nanogrid::GridMap nm({"src", "dst"});
  nm.setGeometry(nanogrid::Length(20.0, 20.0), resolution);
  nm["src"].setRandom();
  nm["dst"].setZero();

  struct Offset { int dr, dc; };
  std::vector<Offset> manualOffsets;
  {
    int rCells = static_cast<int>(std::ceil(radius / resolution));
    double radSq = radius * radius;
    for (int dr = -rCells; dr <= rCells; ++dr)
      for (int dc = -rCells; dc <= rCells; ++dc)
        if (resolution * resolution * (dr * dr + dc * dc) <= radSq)
          manualOffsets.push_back({dr, dc});
  }

  const int rows = nm.getSize()(0);
  const int cols = nm.getSize()(1);
  const int sr = nm.getStartIndex()(0);
  const int sc = nm.getStartIndex()(1);

  std::vector<BenchResult> results;

  results.push_back({"Manual offset loop",
     bench([&]() {
       const auto& src = nm["src"];
       auto& dst = nm["dst"];
       for (auto cell : nm) {
         float maxVal = src(cell.index);
         for (auto& off : manualOffsets) {
           int nr = cell.row + off.dr, nc = cell.col + off.dc;
           if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
           int pr = (nr + sr) % rows, pc = (nc + sc) % cols;
           Eigen::Index idx = static_cast<Eigen::Index>(pc) * rows + pr;
           float v = src(idx);
           maxVal = maxVal > v ? maxVal : v;
         }
         dst(cell.index) = maxVal;
       }
     })});

  auto reg = nm.kernel(radius);
  results.push_back({"nanoGrid::neighbors(region)",
     bench([&]() {
       const auto& src = nm["src"];
       auto& dst = nm["dst"];
       for (auto cell : nm) {
         float maxVal = src(cell.index);
         for (auto n : nm.neighbors(cell, reg)) {
           float v = src(n.index);
           maxVal = maxVal > v ? maxVal : v;
         }
         dst(cell.index) = maxVal;
       }
     })});

  printResults("Neighbors r=2cells (" + std::to_string(N) + "x" + std::to_string(N) + ")", results);
}

int main() {
  std::cout << "nanoGrid Spatial Iteration Benchmark\n"
            << std::string(60, '=') << "\n";
  bench_submap(1000);
  bench_circle(1000);
  bench_neighbors(500);
  bench_neighbors(1000);
  return 0;
}
