# CommonRoad Search - AI Coding Instructions

## Project Overview

This is an educational motion planning framework for autonomous vehicles, used in TUM's "Fundamentals of Artificial Intelligence" course. Students implement heuristic functions and search algorithms to solve motion planning problems using motion primitives.

## Architecture

### Core Components

- **SMP/motion_planner/search_algorithms/**: Search algorithm implementations
  - `base_class.py` - `SearchBaseClass` with 50+ helper methods for heuristics (distance calculations, lanelet operations, collision checking)
  - `best_first_search.py` - `BestFirstSearch`, `GreedyBestFirstSearch`, `AStarSearch`, `UniformCostSearch`
  - `student.py` - **Primary file students edit** - `StudentMotionPlanner` class
  - `student_example.py` - Reference implementation for students

- **SMP/maneuver_automaton/**: Motion primitive management
  - `ManeuverAutomaton` loads vehicle-specific primitives from `automata/` folder
  - Primitives define possible vehicle movements (velocity, steering angle combinations)

- **SMP/batch_processing/**: Scenario evaluation pipeline
  - `batch_processing_config.yaml` - Configuration for planner selection, timeouts, scenarios
  - Supports sequential and parallel processing of multiple scenarios

### Key Data Flow

1. `Scenario` + `PlanningProblem` → loaded from XML files in `scenarios/`
2. `ManeuverAutomaton` provides motion primitives for vehicle type
3. Search algorithm explores primitive combinations via `PriorityNode` tree
4. `reached_goal()` checks if path satisfies goal constraints
5. Solutions written to `outputs/solutions/` as XML

## Student Implementation Pattern

When modifying `student.py`, follow this structure:

```python
class StudentMotionPlanner(GreedyBestFirstSearch):  # Can inherit any search algorithm
    def __init__(self, scenario, planningProblem, automata, plot_config):
        super().__init__(...)
        # Initialize custom attributes (reference paths, goal analysis, etc.)

    def evaluation_function(self, node_current: PriorityNode) -> float:
        # For A* style: return g(n) + h(n)
        # For Greedy: return h(n)
        node_current.priority = self.heuristic_function(node_current)
        return node_current.priority

    def heuristic_function(self, node_current: PriorityNode) -> float:
        # Access path via: node_current.list_paths[-1]
        # Access depth via: node_current.depth_tree
        # Use helper methods from SearchBaseClass
```

## Essential Helper Methods (from `SearchBaseClass`)

| Method | Purpose |
|--------|---------|
| `calc_heuristic_distance(state)` | Distance to goal position |
| `calc_heuristic_lanelet(path)` | Lanelet-based cost, returns `(cost, end_lanelet_id, start_lanelet_id)` |
| `calc_euclidean_distance(node)` | Euclidean distance to goal region |
| `calc_travelled_distance(path)` | Total path length |
| `calc_path_efficiency(path)` | distance / time_steps |
| `calc_angle_to_goal(state)` | Orientation towards goal |
| `calc_orientation_diff(angle1, angle2)` | Difference between orientations |
| `is_goal_in_lane(lanelet_id)` | Check if goal reachable from lanelet |
| `dict_lanelets_costs[lanelet_id]` | Pre-computed cost to goal (-1 = unreachable) |
| `calc_distance_to_nearest_point(polyline, pos)` | Distance to reference polyline |

## Goal State Handling

Goals can specify position, velocity, orientation, and/or time intervals. Handle `Interval` objects:

```python
# Goal state attributes may be Interval objects
if hasattr(self.planningProblem.goal.state_list[0], "velocity"):
    v_goal = self.planningProblem.goal.state_list[0].velocity
    v_mean = (v_goal.start + v_goal.end) / 2

# Time-only goals (survival scenarios) have no position constraint
self.time_only_goal = getattr(gs, "position", None) is None
```

## Vehicle Model

Only the `BMW_320i` vehicle model is used in this project.

## Important: Code Execution Policy

**Never run or execute code.** The user will always handle execution and testing themselves.