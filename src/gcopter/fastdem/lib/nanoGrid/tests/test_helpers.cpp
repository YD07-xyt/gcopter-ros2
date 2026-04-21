/*
 * test_helpers.cpp
 *
 *  Created on: Mar 3, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "test_helpers.hpp"

#include "nanogrid/GridMap.hpp"

// gtest
#include <gtest/gtest.h>

namespace nanogrid_test {

std::mt19937 rndGenerator;

AnalyticalFunctions createFlatWorld(nanogrid::GridMap *map)
{

  AnalyticalFunctions func;

  func.f_ = [](double  /*x*/, double  /*y*/) {
    return 0.0;
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createRationalFunctionWorld(nanogrid::GridMap *map)
{

  AnalyticalFunctions func;

  std::uniform_real_distribution<double> shift(-3.0, 3.0);
  std::uniform_real_distribution<double> scale(1.0, 20.0);
  const double x0 = shift(rndGenerator);
  const double y0 = shift(rndGenerator);
  const double s = scale(rndGenerator);

  func.f_ = [x0, y0,s](double x, double y) {
    return s / (1 + std::pow(x-x0, 2.0) + std::pow(y-y0, 2.0));
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createSecondOrderPolyWorld(nanogrid::GridMap *map)
{

  AnalyticalFunctions func;

  func.f_ = [](double x,double y) {
    return (-x*x -y*y +2.0*x*y +x*x*y*y);
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createSaddleWorld(nanogrid::GridMap *map)
{
  AnalyticalFunctions func;

  func.f_ = [](double x,double y) {
    return (x*x-y*y);
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createSineWorld(nanogrid::GridMap *map)
{

  AnalyticalFunctions func;

  std::uniform_real_distribution<double> Uw(0.1, 4.0);
  const double w1 = Uw(rndGenerator);
  const double w2 = Uw(rndGenerator);
  const double w3 = Uw(rndGenerator);
  const double w4 = Uw(rndGenerator);

  func.f_ = [w1,w2,w3,w4](double x,double y) {
    return std::cos(w1*x) + std::sin(w2*y) + std::cos(w3*x) + std::sin(w4*y);
  };

  fillGridMap(map, func);

  return func;

}

AnalyticalFunctions createTanhWorld(nanogrid::GridMap *map)
{

  AnalyticalFunctions func;

  std::uniform_real_distribution<double> scaling(0.1, 2.0);
  const double s = scaling(rndGenerator);
  func.f_ = [s](double x,double  /*y*/) {
    const double expZ = std::exp(2 *s* x);
    return (expZ - 1) / (expZ + 1);
  };

  fillGridMap(map, func);

  return func;
}

AnalyticalFunctions createGaussianWorld(nanogrid::GridMap *map)
{

  struct Gaussian
  {
    double x0, y0;
    double varX, varY;
    double s;
  };

  AnalyticalFunctions func;

  std::uniform_real_distribution<double> var(0.1, 3.0);
  std::uniform_real_distribution<double> mean(-4.0, 4.0);
  std::uniform_real_distribution<double> scale(-3.0, 3.0);
  constexpr int numGaussians = 3;
  std::array<Gaussian, numGaussians> g;

  for (int i = 0; i < numGaussians; ++i) {
    g.at(i).x0 = mean(rndGenerator);
    g.at(i).y0 = mean(rndGenerator);
    g.at(i).varX = var(rndGenerator);
    g.at(i).varY = var(rndGenerator);
    g.at(i).s = scale(rndGenerator);
  }

  func.f_ = [g](double x,double y) {
    double value = 0.0;
    for (const auto & i : g) {
      const double x0 = i.x0;
      const double y0 = i.y0;
      const double varX = i.varX;
      const double varY = i.varY;
      const double s = i.s;
        value += s * std::exp(-(x-x0)*(x-x0) / (2.0*varX) - (y-y0)*(y-y0) / (2.0 * varY));
    }

    return value;
  };

  fillGridMap(map, func);

  return func;
}

void fillGridMap(nanogrid::GridMap *map, const AnalyticalFunctions &functions)
{
  using nanogrid::DataType;
  using nanogrid::Matrix;

  Matrix& data = (*map)[testLayer];
  for (auto cell : *map) {
    auto pos = map->position(cell);
    if (pos) {
      data(cell.index) = static_cast<DataType>(functions.f_(pos->x(), pos->y()));
    }
  }
}

nanogrid::GridMap createMap(const nanogrid::Length &length, double resolution,
                            const nanogrid::Position &pos)
{
  nanogrid::GridMap map;

  map.setGeometry(length, resolution, pos);
  map.add(testLayer, 0.0);
  map.setFrameId("map");

  return map;
}

std::vector<Point2D> uniformlyDitributedPointsWithinMap(const nanogrid::GridMap &map,
                                                       unsigned int numPoints)
{

  // stay away from the edges
  // on the edges the cubic interp is invalid. Not enough points.
  const double dimX = map.getLength().x() / 2.0 - 3.0 * map.getResolution();
  const double dimY = map.getLength().y() / 2.0 - 3.0 * map.getResolution();
  std::uniform_real_distribution<double> Ux(-dimX, dimX);
  std::uniform_real_distribution<double> Uy(-dimY, dimY);

  std::vector<Point2D> points(numPoints);
  for (auto &point : points) {
    point.x_ = Ux(rndGenerator);
    point.y_ = Uy(rndGenerator);
  }

  return points;
}

} /* namespace nanogrid_test */

