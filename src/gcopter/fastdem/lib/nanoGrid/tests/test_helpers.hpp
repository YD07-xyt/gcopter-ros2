/*
 * test_helpers.hpp
 *
 *  Created on: Mar 3, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once
#include "nanogrid/TypeDefs.hpp"
#include <functional>
#include <vector>
#include <random>

namespace nanogrid {
  class GridMap;
}

namespace nanogrid_test {

/*
 * Name of the layer that is used in all tests.
 * It has no special meaning.
 */
static const std::string testLayer = "test";

/*
 * Class that holds a function pointer to analytical
 * functions used for evaluation. Analytical functions
 * are in the form of f = f(x,y). One could also add
 * derivatives to this class, e.g. for testing the
 * accuracy of the normal estimation.
 */
struct AnalyticalFunctions
{
  std::function<double(double, double)> f_;
};

struct Point2D
{
  double x_ = 0.0;
  double y_ = 0.0;
};

// Random generator engine.
extern std::mt19937 rndGenerator;

// Maximal tolerance when comparing doubles in tests.
const double maxAbsErrorValue = 1e-3;

nanogrid::GridMap createMap(const nanogrid::Length &length, double resolution,
                            const nanogrid::Position &pos);

/*
 * Collections of methods that modify the grid map.
 * All these methods create an analytical function that
 * describes the value of the layer "testLayer" as a function
 * of coordinates.  That can be any mathematical function. Inside the test,
 * sinusoidal, polynomial functions and exponential functions are used.
 * e.g. f(x,y) = sin(x) + cos(y), f(x,y) = exp(-x*x - y*y)
 * Grid map is then filled by evaluating
 * that analytical function over the entire length and width of the
 * grid map. Grid map thus contains spatially sampled mathematical
 * function.
 * Each method returns a structure containing the analytical function.
 */
AnalyticalFunctions createFlatWorld(nanogrid::GridMap *map);
AnalyticalFunctions createRationalFunctionWorld(nanogrid::GridMap *map);
AnalyticalFunctions createSaddleWorld(nanogrid::GridMap *map);
AnalyticalFunctions createSecondOrderPolyWorld(nanogrid::GridMap *map);
AnalyticalFunctions createSineWorld(nanogrid::GridMap *map);
AnalyticalFunctions createTanhWorld(nanogrid::GridMap *map);
AnalyticalFunctions createGaussianWorld(nanogrid::GridMap *map);

/*
 * Iterates over the grid map and fill it with values.
 * values are calculated by evaluating analytical function.
 */
void fillGridMap(nanogrid::GridMap *map, const AnalyticalFunctions &functions);

/*
 * Create numPoints uniformly distributed random points that lie within the grid map.
 */
std::vector<Point2D> uniformlyDitributedPointsWithinMap(const nanogrid::GridMap &map,
                                                        unsigned int numPoints);

}  // namespace nanogrid_test
