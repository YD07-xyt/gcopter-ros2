<h1>
  <img src="./assets/nanoGrid_logo.svg" align="left" height="46px" alt="nanoGrid logo"/>
  <span>nanoGrid</span>
</h1>

[![C++17](https://img.shields.io/badge/C++17-00599C?logo=cplusplus&logoColor=white)](https://github.com/Ikhyeon-Cho/nanoGrid)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-%2328A745)](https://github.com/Ikhyeon-Cho/nanoGrid/blob/main/LICENSE)
![Linux](https://img.shields.io/badge/Linux-555555?style=flat&logo=linux&logoColor=white)
![macOS](https://img.shields.io/badge/macOS-555555?style=flat&logo=apple&logoColor=white)
![Windows](https://img.shields.io/badge/Windows-555555?style=flat&logo=windows&logoColor=white)
[![CI](https://github.com/Ikhyeon-Cho/nanoGrid/actions/workflows/ci.yml/badge.svg)](https://github.com/Ikhyeon-Cho/nanoGrid/actions/workflows/ci.yml)

> Multi-layer grid maps for any C++ project.

nanoGrid is a C++17 library for multi-layer 2.5D grid maps. Each layer — elevation, surface normals, traversability, or any per-cell data — is stored as a named Eigen float matrix on a fixed-resolution grid. It adds a modernized API and faster iteration on top of the [grid_map](https://github.com/ANYbotics/grid_map).

→ [Quick Start](#quick-start)

## Why nanoGrid?

nanoGrid takes `grid_map`'s core and makes it standalone, modernized, and faster:

- **Compact** — Eigen-only dependency.
- **Standalone** — Pure CMake, no ROS. Just add it with `FetchContent` in 3 lines.
- **Modern API** — Less code, fewer mistakes.
- **Faster performance** — Range-based for loop at raw Eigen speed.

  | Method | Time | vs Baseline |
  |--------|------|-------------|
  | **Eigen** bulk operation (SIMD-optimized) | 8.2 ms | **1.0x** |
  | **nanoGrid `for (auto cell : map)`** | **8.3 ms** | **1.0x** |
  | **nanoGrid `for (auto cell : map)`** + grid coordinates | **22.7 ms** | **2.8x** |
  | **grid_map** `GridMapIterator` + `getLinearIndex()` | 83.7 ms | 10.3x |
  | **grid_map** `GridMapIterator` + `operator*()` | 192 ms | 23.5x |
  | **grid_map** `GridMapIterator` + `map.at()` | 614 ms | 75.2x |

  *5000×5000 grid (25M cells), Release build*

## Minimal Example

Create a grid, write cells, and read back by world position:

```cpp
#include <nanogrid/nanogrid.hpp>
using namespace nanogrid;

GridMap map;
map.setGeometry(Length(20.0, 20.0), 0.1);  // 20×20m, 0.1m resolution
map.add("elevation");

// Write — direct Eigen matrix access
auto& elevation = map["elevation"];
for (auto cell : map) {
    elevation(cell.index) = 42.0f;
}

// Read — query by world position
if (auto val = map.get("elevation", Position(1.0, 2.0))) {
    // *val is 42.0f
}
```

---

## Quick Start

### Build

```bash
git clone https://github.com/Ikhyeon-Cho/nanoGrid.git
cd nanoGrid
cmake -B build
cmake --build build
cmake --install build  # optional
```

<details>
<summary>Using nanoGrid in your project?</summary>
<br>

```cmake
find_package(nanoGrid QUIET)
if(NOT nanoGrid_FOUND)
  include(FetchContent)
  FetchContent_Declare(nanoGrid
    GIT_REPOSITORY https://github.com/Ikhyeon-Cho/nanoGrid.git
    GIT_TAG main
  )
  FetchContent_MakeAvailable(nanoGrid)
endif()
target_link_libraries(your_target PUBLIC nanoGrid::nanoGrid)
```

</details>

<details>
<summary>Using with ROS?</summary>
<br>

Just include a single header: `nanogrid/bridge/ros2.hpp`

```cpp
#include <nanogrid/bridge/ros2.hpp>  // or ros1.hpp

auto msg = nanogrid::ros2::toMsg(map);              // publish
auto map = nanogrid::ros2::fromMsg(msg);             // subscribe
auto msg = nanogrid::ros2::toMsg(map, {"elevation"}); // selective layers
auto marker = nanogrid::ros2::toBoundaryMarker(map);  // boundary marker
```

</details>

## API Overview

See [`GridMap.hpp`](include/nanogrid/GridMap.hpp) for the full API.

**Value access — safe reads with `std::optional`:**

```cpp
if (auto val = map.get("elevation", pos)) { /* use *val */ }
if (auto idx = map.index(pos))            { /* use *idx */ }
if (auto pos = map.position(idx))         { /* use *pos */ }
if (auto sub = map.submap(pos, length))   { /* use *sub */ }
```

**Iterating all cells:**

```cpp
auto& elevation = map["elevation"];
for (auto cell : map) {
    elevation(cell.index) = 0.0f;       // linear index — Eigen speed
    if (cell.row == 0 || cell.col == 0)  // logical grid coordinates
        elevation(cell.index) = -1.0f;
}
```

**Spatial queries — rectangular and circular regions:**

```cpp
for (auto cell : map.region(center, Length(5.0, 5.0))) {
    // cells within a rectangular area
}

for (auto cell : map.circle(center, 3.0)) {
    // cells within radius 3.0m
}
```

**Per-cell filtering — apply a kernel to every cell:**

```cpp
auto k = map.kernel(1.5);  // circular neighborhood, radius 1.5m
for (auto cell : map) {
    for (auto nbr : map.neighbors(cell, k)) {
        float d = nbr.dist_sq;  // squared distance to center
        // nbr.index, nbr.row, nbr.col available
    }
}
```

## Acknowledgments

nanoGrid would not exist without the excellent work on [grid_map](https://github.com/ANYbotics/grid_map) by Péter Fankhauser and the teams at ETH Zurich and ANYbotics AG. Their design of the multi-layer grid map data structure has been foundational for robotics research and deployment worldwide.

---

<div align="center">

BSD-3-Clause License © [Ikhyeon Cho](mailto:ikhyeon.c@gmail.com), [ANYbotics AG](https://www.anybotics.com/)

</div>
