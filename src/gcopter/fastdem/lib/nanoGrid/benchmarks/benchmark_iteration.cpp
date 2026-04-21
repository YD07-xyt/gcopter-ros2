/*
 * benchmark_iteration.cpp
 *
 * Compares iteration methods on a 5000x5000 grid (25M cells).
 * Based on the official grid_map iterator_benchmark by ANYbotics.
 */

#include <nanogrid/nanogrid.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <functional>

using namespace nanogrid;
using Clock = std::chrono::high_resolution_clock;

// V4: Eigen built-in
void v4_eigen(GridMap& map) {
  map["to"] = map["to"].cwiseMax(map["from"]);
}

// V5: Raw nested 2D loop (column-major)
void v5_raw_2d(GridMap& map) {
  const auto& src = map["from"];
  auto& dst = map["to"];
  for (Eigen::Index j = 0; j < dst.cols(); ++j) {
    for (Eigen::Index i = 0; i < dst.rows(); ++i) {
      dst(i, j) = dst(i, j) > src(i, j) ? dst(i, j) : src(i, j);
    }
  }
}

// V6: Raw linear loop
void v6_raw_linear(GridMap& map) {
  const auto& src = map["from"];
  auto& dst = map["to"];
  for (Eigen::Index i = 0; i < dst.size(); ++i) {
    dst(i) = dst(i) > src(i) ? dst(i) : src(i);
  }
}

// V7: cells() range-based for
void v7_cells(GridMap& map) {
  const auto& src = map["from"];
  auto& dst = map["to"];
  for (auto cell : map) {
    dst(cell.index) = dst(cell.index) > src(cell.index) ? dst(cell.index) : src(cell.index);
  }
}

// V8: cells() with row/col access
void v8_cells_rowcol(GridMap& map) {
  const auto& src = map["from"];
  auto& dst = map["to"];
  int dummy = 0;
  for (auto cell : map) {
    dst(cell.index) = dst(cell.index) > src(cell.index) ? dst(cell.index) : src(cell.index);
    dummy += cell.row;
  }
  volatile int sink = dummy;
  (void)sink;
}

struct BenchResult {
  std::string name;
  double ms;
};

double bench(GridMap& map, std::function<void(GridMap&)> fn, int warmup = 1, int runs = 5) {
  // Reset data
  for (int i = 0; i < warmup; ++i) {
    map["to"].setRandom();
    fn(map);
  }

  double total = 0.0;
  for (int i = 0; i < runs; ++i) {
    map["to"].setRandom();
    auto t0 = Clock::now();
    fn(map);
    auto t1 = Clock::now();
    total += std::chrono::duration<double, std::milli>(t1 - t0).count();
  }
  return total / runs;
}

int main() {
  // 5000x5000 grid = 25M cells
  const int N = 5000;
  const double resolution = 20.0 / N;

  GridMap map;
  map.setGeometry(Length(20.0, 20.0), resolution);
  map.add("from");
  map.add("to");
  map["from"].setRandom();
  map["to"].setRandom();

  std::cout << "nanoGrid Iteration Benchmark (" << N << "x" << N
            << " = " << N * N / 1000000 << "M cells)\n";
  std::cout << std::string(60, '-') << "\n";

  std::vector<BenchResult> results;
  results.push_back({"V4: Eigen cwiseMax()",           bench(map, v4_eigen)});
  results.push_back({"V5: Raw 2D loop",               bench(map, v5_raw_2d)});
  results.push_back({"V6: Raw linear loop",           bench(map, v6_raw_linear)});
  results.push_back({"V7: cells() range-based for",   bench(map, v7_cells)});
  results.push_back({"V8: cells() + row/col access",  bench(map, v8_cells_rowcol)});

  double fastest = results[0].ms;
  for (auto& r : results) fastest = std::min(fastest, r.ms);

  std::cout << std::left << std::setw(40) << "Method"
            << std::right << std::setw(10) << "Time (ms)"
            << std::setw(10) << "Ratio" << "\n";
  std::cout << std::string(60, '-') << "\n";

  for (auto& r : results) {
    std::cout << std::left << std::setw(40) << r.name
              << std::right << std::setw(10) << std::fixed << std::setprecision(1) << r.ms
              << std::setw(9) << std::setprecision(1) << (r.ms / fastest) << "x\n";
  }

  return 0;
}
