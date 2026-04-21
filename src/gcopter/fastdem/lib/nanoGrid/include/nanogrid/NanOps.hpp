// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025 Ikhyeon Cho. All rights reserved.
//
// NaN-aware reduction functions for Eigen matrices.

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <limits>

namespace nanogrid {

/// Count of finite (non-NaN) elements.
template <typename Derived>
typename Derived::Scalar numberOfFinites(const Eigen::MatrixBase<Derived>& m) {
  return static_cast<typename Derived::Scalar>(
      (m.array() == m.array()).count());
}

/// Sum of all finite elements (NaN treated as 0).
/// Returns NaN if all elements are NaN.
template <typename Derived>
typename Derived::Scalar sumOfFinites(const Eigen::MatrixBase<Derived>& m) {
  using Scalar = typename Derived::Scalar;
  const auto a = m.array();
  const auto finite = (a == a);
  if (finite.any()) {
    return finite.select(a, Scalar(0)).sum();
  }
  return std::numeric_limits<Scalar>::quiet_NaN();
}

/// Mean of finite elements.
template <typename Derived>
typename Derived::Scalar meanOfFinites(const Eigen::MatrixBase<Derived>& m) {
  return sumOfFinites(m) / numberOfFinites(m);
}

/// Minimum finite coefficient.
/// Returns NaN if all elements are NaN.
template <typename Derived>
typename Derived::Scalar minCoeffOfFinites(const Eigen::MatrixBase<Derived>& m) {
  using Scalar = typename Derived::Scalar;
  const auto a = m.array();
  const auto finite = (a == a);
  if (finite.any()) {
    return finite.select(a, std::numeric_limits<Scalar>::infinity()).minCoeff();
  }
  return std::numeric_limits<Scalar>::quiet_NaN();
}

/// Maximum finite coefficient.
/// Returns NaN if all elements are NaN.
template <typename Derived>
typename Derived::Scalar maxCoeffOfFinites(const Eigen::MatrixBase<Derived>& m) {
  using Scalar = typename Derived::Scalar;
  const auto a = m.array();
  const auto finite = (a == a);
  if (finite.any()) {
    return finite.select(a, -std::numeric_limits<Scalar>::infinity()).maxCoeff();
  }
  return std::numeric_limits<Scalar>::quiet_NaN();
}

/// Returns a mask where finite elements are 1.0 and NaN elements are 0.0.
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
              Derived::ColsAtCompileTime>
isFinite(const Eigen::MatrixBase<Derived>& m) {
  const auto a = m.array();
  using Scalar = typename Derived::Scalar;
  return (Scalar(1) - (a != a).template cast<Scalar>()).matrix();
}

}  // namespace nanogrid
