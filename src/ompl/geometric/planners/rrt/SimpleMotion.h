#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_SIMPLE_MOTION_
#define OMPL_GEOMETRIC_PLANNERS_RRT_SIMPLE_MOTION_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/rrt/SimpleMotion.h"

#include <limits>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <list>

namespace ompl
{
    namespace geometric
    {
      /** \brief Representation of a motion */
      class SimpleMotion
      {
      public:
          /** \brief Constructor that allocates memory for the state. This constructor automatically allocates
            * memory for \e state, \e cost, and \e incCost */
          SimpleMotion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr), inGoal(false)
          {
          }

          ~SimpleMotion() = default;

          /** \brief The state contained by the motion */
          base::State *state;

          /** \brief The parent motion in the exploration tree */
          SimpleMotion *parent;

          /** \brief Set to true if this vertex is in the goal region */
          bool inGoal;

          /** \brief The cost up to this motion */
          base::Cost cost;

          /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance
            * computations in the updateChildCosts() method) */
          base::Cost incCost;

          /** \brief The set of motions descending from the current motion */
          std::vector<SimpleMotion *> children;
      };
    }
}

#endif