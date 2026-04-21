/*
 * NanOpsTest.cpp
 *
 * Tests for NaN-aware reduction free functions.
 */

#include "nanogrid/NanOps.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

using Eigen::Matrix;
using namespace nanogrid;

TEST(NanOps, numberOfFinites)
{
  Eigen::Matrix3f matrix(Eigen::Matrix3f::Ones());
  matrix(0, 0) = NAN;
  matrix(1, 0) = NAN;
  EXPECT_EQ(7, numberOfFinites(matrix));

  Matrix<double, 13, 10> matrix2;
  matrix2.setOnes();
  EXPECT_EQ(matrix2.rows() * matrix2.cols(), numberOfFinites(matrix2));

  Matrix<double, 13, 10> matrix3;
  matrix3.setConstant(NAN);
  matrix3.col(3).setConstant(0.0);
  EXPECT_EQ(matrix3.rows(), numberOfFinites(matrix3));
}

TEST(NanOps, sumOfFinites)
{
  Matrix<double, 7, 18> matrix;
  matrix.setRandom();
  EXPECT_NEAR(matrix.sum(), sumOfFinites(matrix), 1e-10);
  double finiteSum = matrix.sum() - matrix(0, 0) - matrix(1, 2) - matrix(3, 6) - matrix(6, 12);
  matrix(0, 0) = NAN;
  matrix(1, 2) = NAN;
  matrix(3, 6) = NAN;
  matrix(6, 12) = NAN;
  EXPECT_NEAR(finiteSum, sumOfFinites(matrix), 1e-10);
  matrix.setConstant(NAN);
  EXPECT_TRUE(std::isnan(sumOfFinites(matrix)));
  matrix(5, 7) = 1.0;
  EXPECT_NEAR(1.0, sumOfFinites(matrix), 1e-10);
}

TEST(NanOps, meanOfFinites)
{
  Eigen::Matrix3f matrix(Eigen::Matrix3f::Ones());
  matrix(0, 0) = NAN;
  matrix(1, 1) = NAN;
  EXPECT_DOUBLE_EQ(1.0, meanOfFinites(matrix));

  Matrix<double, 13, 10> matrix2;
  matrix2.setRandom();
  EXPECT_NEAR(matrix2.mean(), meanOfFinites(matrix2), 1e-10);
}

TEST(NanOps, minCoeffOfFinites)
{
  Matrix<double, 7, 18> matrix;
  matrix.setRandom();
  double min = matrix.minCoeff();
  EXPECT_NEAR(min, minCoeffOfFinites(matrix), 1e-10);

  int i;
  int j;
  matrix.maxCoeff(&i, &j);
  matrix(i, j) = NAN;
  EXPECT_NEAR(min, minCoeffOfFinites(matrix), 1e-10);

  matrix.setConstant(NAN);
  EXPECT_TRUE(std::isnan(minCoeffOfFinites(matrix)));
  matrix(i, j) = -1.0;
  EXPECT_NEAR(-1.0, minCoeffOfFinites(matrix), 1e-10);
}

TEST(NanOps, maxCoeffOfFinites)
{
  Matrix<double, 7, 18> matrix;
  matrix.setRandom();
  double max = matrix.maxCoeff();
  EXPECT_NEAR(max, maxCoeffOfFinites(matrix), 1e-10);

  int i;
  int j;
  matrix.minCoeff(&i, &j);
  matrix(i, j) = NAN;
  EXPECT_NEAR(max, maxCoeffOfFinites(matrix), 1e-10);

  matrix.setConstant(NAN);
  EXPECT_TRUE(std::isnan(maxCoeffOfFinites(matrix)));
  matrix(i, j) = -1.0;
  EXPECT_NEAR(-1.0, maxCoeffOfFinites(matrix), 1e-10);
}
