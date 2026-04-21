/*
 * TypeDefs.hpp
 *
 *  Created on: March 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

// Eigen
#pragma once

#include <Eigen/Core>

namespace nanogrid {

  using Matrix = Eigen::MatrixXf;
  using DataType = Matrix::Scalar;
  using Position = Eigen::Vector2d;
  using Vector = Eigen::Vector2d;
  using Position3 = Eigen::Vector3d;
  using Vector3 = Eigen::Vector3d;
  using Index = Eigen::Array2i;
  using Size = Eigen::Array2i;
  using Length = Eigen::Array2d;
  using Time = uint64_t;

}  // namespace nanogrid

