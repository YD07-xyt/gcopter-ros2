/*
 * GridMap.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 *
 *  Modified on: Jan, 2026
 *      Author: Ikhyeon Cho
 *  Institute: Korea University, Seoul
 */

#pragma once

#include "nanogrid/TypeDefs.hpp"

namespace nanogrid { class BufferRegion; }

// STL
#include <cmath>
#include <optional>
#include <unordered_map>
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace nanogrid {

/*!
 * Grid map managing multiple overlaying maps holding float values.
 * Data structure implemented as two-dimensional circular buffer so map
 * can be moved efficiently.
 *
 * Data is defined with string keys. Examples are:
 * - "elevation"
 * - "variance"
 * - "color"
 * - "quality"
 * - "surface_normal_x", "surface_normal_y", "surface_normal_z"
 * etc.
 */
class GridMap {
 public:
  // Type traits for use with template methods/classes using GridMap as a template parameter.
  typedef nanogrid::DataType DataType;
  typedef nanogrid::Matrix Matrix;

  /*!
   * Constructor.
   * @param layers a vector of strings containing the definition/description of the data layer.
   */
  explicit GridMap(const std::vector<std::string>& layers);

  /*!
   * Emtpy constructor.
   */
  GridMap();

  /*!
   * Default copy assign and copy constructors.
   */
  GridMap(const GridMap&) = default;
  GridMap& operator=(const GridMap&) = default;
  GridMap(GridMap&&) = default;
  GridMap& operator=(GridMap&&) = default;

  /*!
   * Destructor.
   */
  virtual ~GridMap() = default;

  /*!
   * Set the geometry of the grid map. Clears all the data.
   * @param length the side lengths in x, and y-direction of the grid map [m].
   * @param resolution the cell size in [m/cell].
   * @param position the 2d position of the grid map in the grid map frame [m].
   */
  void setGeometry(const Length& length, const double resolution, const Position& position = Position::Zero());

  /*!
   * Add a new empty data layer.
   * @param layer the name of the layer.
   * @value value the value to initialize the cells with.
   */
  void add(const std::string& layer, const float value = NAN);

  /*!
   * Add a new data layer (if the layer already exists, overwrite its data, otherwise add layer and data).
   * @param layer the name of the layer.
   * @param data the data to be added.
   */
  void add(const std::string& layer, const Matrix& data);

  /*!
   * Checks if data layer exists.
   * @param layer the name of the layer.
   * @return true if layer exists, false otherwise.
   */
  [[nodiscard]] bool exists(const std::string& layer) const;

  /*!
   * Returns the grid map data for a layer as matrix.
   * @param layer the name of the layer to be returned.
   * @return grid map data as matrix.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  const Matrix& get(const std::string& layer) const;

  /*!
   * Returns the grid map data for a layer as non-const. Use this method
   * with care!
   * @param layer the name of the layer to be returned.
   * @return grid map data.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  Matrix& get(const std::string& layer);

  /*!
   * Returns the grid map data for a layer as matrix.
   * @param layer the name of the layer to be returned.
   * @return grid map data as matrix.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  const Matrix& operator[](const std::string& layer) const;

  /*!
   * Returns the grid map data for a layer as non-const. Use this method
   * with care!
   * @param layer the name of the layer to be returned.
   * @return grid map data.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  Matrix& operator[](const std::string& layer);

  /*!
   * Removes a layer from the grid map.
   * @param layer the name of the layer to be removed.
   * @return true if successful.
   */
  [[nodiscard]] bool erase(const std::string& layer);

  /*!
   * Gets the names of the layers.
   * @return the names of the layers.
   */
  const std::vector<std::string>& getLayers() const;

  /*!
   * Checks if another grid map contains the same layers as this grid map.
   * The other grid map could contain more layers than the checked ones.
   * Does not check the selection of basic layers.
   * @param other the other grid map.
   * @return true if the other grid map has the same layers, false otherwise.
   */
  [[nodiscard]] bool hasSameLayers(const nanogrid::GridMap& other) const;

  /*!
   * Get cell data for requested index.
   * @param layer the name of the layer to be accessed.
   * @param index the requested index.
   * @return the data of the cell.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  float& at(const std::string& layer, const Index& index);

  /*!
   * Get cell data for requested index. Const version form above.
   * @param layer the name of the layer to be accessed.
   * @param index the requested index.
   * @return the data of the cell.
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  float at(const std::string& layer, const Index& index) const;

  // ============================================================
  // Modern C++17 API
  // ============================================================

  /*!
   * Gets the corresponding cell index for a position.
   * @param position the requested position.
   * @return the index if position is inside the map, std::nullopt otherwise.
   */
  [[nodiscard]] std::optional<Index> index(const Position& position) const;

  /*!
   * Gets the 2d position of cell specified by the index.
   * @param index the index of the requested cell.
   * @return the position if index is within range, std::nullopt otherwise.
   */
  [[nodiscard]] std::optional<Position> position(const Index& index) const;

  /*!
   * Gets the 3d position of a data point (x, y of cell position & cell value as z).
   * @param layer the name of the layer to be accessed.
   * @param index the index of the requested cell.
   * @return the 3d position if valid data available, std::nullopt otherwise.
   */
  [[nodiscard]] std::optional<Position3> position3(const std::string& layer, const Index& index) const;

  /*!
   * Gets the 3d vector of three layers with suffixes 'x', 'y', and 'z'.
   * @param layerPrefix the prefix for the layer to get as vector.
   * @param index the index of the requested cell.
   * @return the vector if valid data available, std::nullopt otherwise.
   */
  [[nodiscard]] std::optional<Vector3> vector3(const std::string& layerPrefix, const Index& index) const;

  /*!
   * Gets cell value at position, combining boundary check, index lookup, and validity check.
   * @param layer the name of the layer to be accessed.
   * @param position the requested position.
   * @return the cell value if position is inside the map and the cell is valid, std::nullopt otherwise.
   */
  [[nodiscard]] std::optional<float> get(const std::string& layer, const Position& position) const;

  /*!
   * Gets cell value at index with validity check.
   * @param layer the name of the layer to be accessed.
   * @param index the requested index.
   * @return the cell value if finite, std::nullopt otherwise.
   */
  [[nodiscard]] std::optional<float> get(const std::string& layer, const Index& index) const;

  /*!
   * Gets a submap from the map.
   * @param position the requested position of the submap (usually the center).
   * @param length the requested length of the submap.
   * @return the submap if successful, std::nullopt otherwise.
   */
  [[nodiscard]] std::optional<GridMap> submap(const Position& position, const Length& length) const;

  /// Lightweight submap geometry query (no data copy).
  struct SubRegion { Index startIndex; Size size; };
  [[nodiscard]] std::optional<SubRegion> subRegion(const Position& position, const Length& length) const;

  // ============================================================

  /*!
   * Check if position is within the map boundaries.
   * @param position the position to be checked.
   * @return true if position is within map, false otherwise.
   */
  [[nodiscard]] bool isInside(const Position& position) const;

  /*!
   * Checks if any layer has a valid (finite) value at the given index.
   * @param index the index to check.
   * @return true if any layer has finite data at this cell, false otherwise.
   */
  [[nodiscard]] bool isValid(const Index& index) const;

  /*!
   * Checks if cell at index is a valid (finite) for a certain layer.
   * @param index the index to check.
   * @param layer the name of the layer to be checked for validity.
   * @return true if cell is valid, false otherwise.
   */
  [[nodiscard]] bool isValid(const Index& index, const std::string& layer) const;

  /*!
   * Checks if cell at index is a valid (finite) for certain layers.
   * @param index the index to check.
   * @param layers the layers to be checked for validity.
   * @return true if cell is valid, false otherwise.
   */
  [[nodiscard]] bool isValid(const Index& index, const std::vector<std::string>& layers) const;

  /*!
   * Apply isometric transformation (rotation + offset) to grid map and returns the transformed map.
   * Note: The returned map may not have the same length since it's geometric description contains
   * Note: The transformation will only be applied to the height layer of the grid map, other layers will remain untouched.
   * the original map.
   * @param[in] transform the requested transformation to apply.
   * @param[in] heightLayerName the height layer of the map.
   * @param[in] newFrameId frame index of the new map.
   * @param[in] sampleRatio if zero or negative, no in-painting is used to fill missing points due to sparsity of the map. Otherwise,
   *            four points are sampled around each grid cell to make sure that at least one of those points map to a new grid cell.
   *            A sampleRatio of 1 corresponds to the the resolution of the grid map.
   * @return transformed map.
   * @throw std::out_of_range if no map layer with name `heightLayerName` is present.
   */
  GridMap getTransformedMap(const Eigen::Isometry3d& transform, const std::string& heightLayerName, const std::string& newFrameId,
                            const double sampleRatio = 0.0) const;

  /*!
   * Set the position of the grid map.
   * Note: This method does not change the data stored in the grid map and
   * is complementary to the `move(...)` method. For a comparison between
   * the `setPosition` and the `move` method, see the `move_demo_node.cpp`
   * file of the `grid_map_demos` package.
   * @param position the 2d position of the grid map in the grid map frame [m].
   */
  void setPosition(const Position& position);

  /*!
   * Move the grid map w.r.t. to the grid map frame. Use this to move the grid map
   * boundaries without moving the grid map data. Takes care of all the data handling,
   * such that the grid map data is stationary in the grid map frame.
   * @param position the new location of the grid map in the map frame.
   * @return true if map has been moved, false otherwise.
   */
  bool move(const Position& position);

  /*!
   * Adds data from an other grid map to this grid map
   * @param other the grid map to take data from.
   * @param extendMap if true the grid map is resized that the other map fits within.
   * @param overwriteData if true the new data replaces the old values, else only invalid cells are updated.
   * @param copyAllLayer if true all layers are used to add data.
   * @param layers the layers that are copied if not all layers are used.
   * @return true if successful.
   */
  bool addDataFrom(const GridMap& other, bool extendMap, bool overwriteData, bool copyAllLayers,
                   std::vector<std::string> layers = std::vector<std::string>());

  /*!
   * Extends the size of the grip map such that the other grid map fits within.
   * @param other the grid map to extend the size to.
   * @return true if successful.
   */
  bool extendToInclude(const GridMap& other);

  /*!
   * Clears all cells (set to NAN) for a layer.
   * @param layer the layer to be cleared.
   */
  void clear(const std::string& layer);

  /*!
   * Clears all cells of all layers.
   * Header information (geometry etc.) remains valid.
   */
  void clearAll();

  /*!
   * Set the timestamp of the grid map.
   * @param timestamp the timestamp to set (in  nanoseconds).
   */
  void setTimestamp(const Time timestamp);

  /*!
   * Get the timestamp of the grid map.
   * @return timestamp in nanoseconds.
   */
  Time getTimestamp() const;

  /*!
   * Resets the timestamp of the grid map (to zero).
   */
  void resetTimestamp();

  /*!
   * Set the frame id of the grid map.
   * @param frameId the frame id to set.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Get the frameId of the grid map.
   * @return frameId.
   */
  const std::string& getFrameId() const;

  /*!
   * Get the side length of the grid map.
   * @return side length of the grid map.
   */
  const Length& getLength() const;

  /*!
   * Get the 2d position of the grid map in the grid map frame.
   * @return position of the grid map in the grid map frame.
   */
  const Position& getPosition() const;

  /*!
   * Get the resolution of the grid map.
   * @return resolution of the grid map in the xy plane [m/cell].
   */
  double getResolution() const;

  /*!
   * Get the grid map size (rows and cols of the data structure).
   * @return grid map size.
   */
  const Size& getSize() const;

  /*!
   * Set the start index of the circular buffer.
   * Use this method with caution!
   * @return buffer start index.
   */
  void setStartIndex(const Index& startIndex);

  /*!
   * Get the start index of the circular buffer.
   * @return buffer start index.
   */
  const Index& getStartIndex() const;

  /*!
   * Checks if the buffer is at start index (0,0).
   * @return true if buffer is at default start index.
   */
  [[nodiscard]] bool isDefaultStartIndex() const;

  /*!
   * Rearranges data such that the buffer start index is at (0,0).
   */
  void convertToDefaultStartIndex();

  /*!
   * Calculates the closest point to positionOutMap that is in the grid map.
   * If positionOutMap is already in the grid map, that position is returned.
   * @param[in] position position that should be approached as close as possible.
   * @return position in map.
   */
  Position getClosestPositionInMap(const Position& position) const;

  // ============================================================
  // Cell descriptor
  // ============================================================

  /// Lightweight cell descriptor yielded during iteration.
  /// Carries all index representations so that GridMap methods can
  /// pick the most efficient one internally.
  struct Cell {
    Eigen::Index index;  ///< Column-major linear index — use with data(cell.index).
    int row;             ///< Logical grid row (0..rows-1, buffer-independent).
    int col;             ///< Logical grid column (0..cols-1, buffer-independent).
    int bufRow;          ///< Buffer (physical) row in the Eigen matrix.
    int bufCol;          ///< Buffer (physical) column in the Eigen matrix.
  };

  // ============================================================
  // Cell-based accessors
  // ============================================================

  /// Get cell value via Cell descriptor (fastest — linear index access).
  float& at(const std::string& layer, const Cell& cell);
  float  at(const std::string& layer, const Cell& cell) const;

  /// Get the 2d position of a cell.
  [[nodiscard]] std::optional<Position> position(const Cell& cell) const;

  /// Check if any layer has a valid (finite) value at the cell.
  [[nodiscard]] bool isValid(const Cell& cell) const;

  /// Check if a specific layer has a valid (finite) value at the cell.
  [[nodiscard]] bool isValid(const Cell& cell, const std::string& layer) const;

  // ============================================================
  // Range-based iteration: for (auto cell : map) { ... }
  // ============================================================

  struct CellIterator {
    Eigen::Index i;
    int physRow, physCol;
    int rows, cols, startRow, startCol;

    Cell operator*() const {
      int logRow = physRow - startRow;
      if (logRow < 0) logRow += rows;
      int logCol = physCol - startCol;
      if (logCol < 0) logCol += cols;
      return {i, logRow, logCol, physRow, physCol};
    }

    CellIterator& operator++() {
      ++i;
      if (++physRow >= rows) {
        physRow = 0;
        ++physCol;
      }
      return *this;
    }

    bool operator!=(const CellIterator& o) const { return i != o.i; }
  };

  struct CellRange {
    Eigen::Index size;
    int rows, cols, startRow, startCol;

    CellIterator begin() const { return {0, 0, 0, rows, cols, startRow, startCol}; }
    CellIterator end() const { return {size, 0, 0, rows, cols, startRow, startCol}; }
  };

  /// Iterate all cells. Prefer `for (auto cell : map)`.
  CellRange cells() const {
    return {static_cast<Eigen::Index>(size_.prod()),
            size_(0), size_(1), startIndex_(0), startIndex_(1)};
  }

  /// Range-based for loop support.
  CellIterator begin() const { return cells().begin(); }
  CellIterator end() const { return cells().end(); }

  // ---------------------------------------------------------------------------
  // Region iteration (rectangular world-coordinate area)
  // ---------------------------------------------------------------------------

  struct RegionIterator {
    int logRow, logCol;
    int startRow, endRow, startCol, endCol;
    int rows, cols, bufStartRow, bufStartCol;

    Cell operator*() const {
      int physRow = (logRow + bufStartRow) % rows;
      int physCol = (logCol + bufStartCol) % cols;
      Eigen::Index i = static_cast<Eigen::Index>(physCol) * rows + physRow;
      return {i, logRow, logCol, physRow, physCol};
    }

    RegionIterator& operator++() {
      if (++logRow >= endRow) {
        logRow = startRow;
        ++logCol;
      }
      return *this;
    }

    bool operator!=(const RegionIterator& o) const {
      return logCol != o.logCol || logRow != o.logRow;
    }
  };

  struct RegionRange {
    int startRow, endRow, startCol, endCol;
    int rows, cols, bufStartRow, bufStartCol;

    RegionIterator begin() const {
      if (startRow >= endRow || startCol >= endCol) return end();
      return {startRow, startCol, startRow, endRow, startCol, endCol,
              rows, cols, bufStartRow, bufStartCol};
    }

    RegionIterator end() const {
      return {startRow, endCol, startRow, endRow, startCol, endCol,
              rows, cols, bufStartRow, bufStartCol};
    }
  };

  /// Iterate cells within a rectangular world-coordinate region.
  /// @param center world-frame position of the region center [m].
  /// @param size side lengths of the region in x and y [m].
  RegionRange region(const Position& center, const Length& size) const;

  // ---------------------------------------------------------------------------
  // Circle iteration
  // ---------------------------------------------------------------------------

  struct CircleIterator {
    RegionIterator it, regionEnd;
    double centerRow, centerCol;
    double radiusSq, resolution;

    Cell operator*() const { return *it; }

    CircleIterator& operator++() {
      do { ++it; } while (it != regionEnd && !isInside());
      return *this;
    }

    bool operator!=(const CircleIterator& o) const { return it != o.it; }

    bool isInside() const {
      auto cell = *it;
      double dr = (cell.row - centerRow) * resolution;
      double dc = (cell.col - centerCol) * resolution;
      return dr * dr + dc * dc <= radiusSq;
    }
  };

  struct CircleRange {
    RegionRange rgn;
    double centerRow, centerCol;
    double radiusSq, resolution;

    CircleIterator begin() const {
      auto b = rgn.begin();
      auto e = rgn.end();
      CircleIterator cit{b, e, centerRow, centerCol, radiusSq, resolution};
      if (b != e && !cit.isInside()) ++cit;
      return cit;
    }

    CircleIterator end() const {
      auto e = rgn.end();
      return {e, e, centerRow, centerCol, radiusSq, resolution};
    }
  };

  /// Iterate cells within a circular world-coordinate region.
  /// @param center world-frame position of the circle center [m].
  /// @param radius circle radius [m].
  CircleRange circle(const Position& center, double radius) const;

  // ---------------------------------------------------------------------------
  // Precomputed neighborhood
  // ---------------------------------------------------------------------------

  /// Neighbor descriptor: Cell + squared distance to center.
  struct Neighbor : Cell {
    float dist_sq;  ///< Squared distance in meters from the center cell.
  };

  struct Kernel {
    struct Entry { int dr, dc; float dist_sq; };
    std::vector<Entry> entries;
    int minDr = 0, maxDr = 0, minDc = 0, maxDc = 0;
  };

  /// Precompute a circular neighborhood kernel.
  /// @param radius neighborhood radius [m].
  Kernel kernel(double radius) const;

  /// Precompute a rectangular neighborhood kernel.
  /// @param window neighborhood size in cells (rows, cols).
  Kernel kernel(const Size& window) const;

  // ---------------------------------------------------------------------------
  // Neighbor iteration using precomputed Kernel
  // ---------------------------------------------------------------------------

  struct NeighborIterator {
    const Kernel::Entry* ptr;
    const Kernel::Entry* end;
    int centerRow, centerCol;
    int rows, cols, bufStartRow, bufStartCol;
    bool allValid;

    Neighbor operator*() const {
      int logRow = centerRow + ptr->dr;
      int logCol = centerCol + ptr->dc;
      int physRow = (logRow + bufStartRow) % rows;
      int physCol = (logCol + bufStartCol) % cols;
      Eigen::Index i = static_cast<Eigen::Index>(physCol) * rows + physRow;
      return {{i, logRow, logCol, physRow, physCol}, ptr->dist_sq};
    }

    NeighborIterator& operator++() {
      ++ptr;
      if (!allValid) {
        while (ptr != end && !isValid()) ++ptr;
      }
      return *this;
    }

    bool operator!=(const NeighborIterator& o) const { return ptr != o.ptr; }

   private:
    bool isValid() const {
      int r = centerRow + ptr->dr;
      int c = centerCol + ptr->dc;
      return r >= 0 && r < rows && c >= 0 && c < cols;
    }
  };

  struct NeighborRange {
    const Kernel::Entry* data;
    const Kernel::Entry* dataEnd;
    int centerRow, centerCol;
    int rows, cols, bufStartRow, bufStartCol;
    bool allValid;

    NeighborIterator begin() const {
      NeighborIterator it{data,    dataEnd,     centerRow, centerCol,
                          rows,    cols,        bufStartRow, bufStartCol,
                          allValid};
      if (!allValid && data != dataEnd && !isValid(data)) ++it;
      return it;
    }

    NeighborIterator end() const {
      return {dataEnd, dataEnd, centerRow, centerCol,
              rows,    cols,    bufStartRow, bufStartCol, allValid};
    }

   private:
    bool isValid(const Kernel::Entry* p) const {
      int r = centerRow + p->dr;
      int c = centerCol + p->dc;
      return r >= 0 && r < rows && c >= 0 && c < cols;
    }
  };

  /// Iterate neighbors of a cell using a precomputed Kernel.
  /// Build a Kernel once with kernel(), then reuse it for every cell.
  NeighborRange neighbors(const Cell& cell, const Kernel& nbr) const;

 private:
  // Internal helpers.
  bool getIndex(const Position& position, Index& index) const;
  bool getPosition(const Index& index, Position& position) const;
  GridMap getSubmap(const Position& position, const Length& length, bool& isSuccess) const;
  GridMap getSubmap(const Position& position, const Length& length, Index& indexInSubmap, bool& isSuccess) const;
  bool move(const Position& position, std::vector<BufferRegion>& newRegions);

  bool isValid(DataType value) const;

  /*!
   * Clear a number of columns of the grid map.
   * @param index the left index for the columns to be reset.
   * @param nCols the number of columns to reset.
   */
  void clearCols(unsigned int index, unsigned int nCols);

  /*!
   * Clear a number of rows of the grid map.
   * @param index the upper index for the rows to be reset.
   * @param nRows the number of rows to reset.
   */
  void clearRows(unsigned int index, unsigned int nRows);

  /*!
   * Resize the buffer.
   * @param bufferSize the requested buffer size.
   */
  void resize(const Index& bufferSize);

  //! Frame id of the grid map.
  std::string frameId_;

  //! Timestamp of the grid map (nanoseconds).
  Time timestamp_;

  //! Grid map data stored as layers of matrices.
  std::unordered_map<std::string, Matrix> data_;

  //! Names of the data layers.
  std::vector<std::string> layers_;

  //! Side length of the map in x- and y-direction [m].
  Length length_;

  //! Map resolution in xy plane [m/cell].
  double resolution_;

  //! Map position in the grid map frame [m].
  Position position_;

  //! Size of the buffer (rows and cols of the data structure).
  Size size_;

  //! Circular buffer start indices.
  Index startIndex_;
};

}  // namespace nanogrid
