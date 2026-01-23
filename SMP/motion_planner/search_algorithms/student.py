import numpy as np
from sys import maxsize as INT_MAX
from SMP.motion_planner.node import PriorityNode

from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.search_algorithms.best_first_search import GreedyBestFirstSearch
import commonroad_route_planner.fast_api.fast_api as cr_rp_api

class StudentMotionPlanner(GreedyBestFirstSearch):
    """
    Motion planner implementation by students.
    Note that you may inherit from any given motion planner as you wish, or come up with your own planner.
    Here as an example, the planner is inherited from the GreedyBestFirstSearch planner.
    """

    def __init__(
        self, scenario, planningProblem, automata, plot_config=DefaultPlotConfig
    ):
        super().__init__(
            scenario=scenario,
            planningProblem=planningProblem,
            automaton=automata,
            plot_config=plot_config,
        )

        # Calculate a reference path for the planning problem
        try:
            reference_path_object = cr_rp_api.generate_reference_path_from_lanelet_network_and_planning_problem(
                       lanelet_network=scenario.lanelet_network,
                        planning_problem=self.planningProblem
            ) 
            #print("Reference path generated successfully using Fast API.")
        except Exception as e:
            print(f"Error generating reference path using Fast API: {e}")
            reference_path_object = None

        # Save on the instance and safely extract coordinates if available
        self.reference_path_object = reference_path_object
        self.reference_path_coords = (
            reference_path_object.reference_path
            if reference_path_object is not None and hasattr(reference_path_object, "reference_path")
            else None
        )

        if self.reference_path_coords is None:
            raise ValueError("Reference path could not be generated.")

        # Check if goal is just time interval
        self.time_only_goal = True
        self.is_time_dependent = False
        self.survival_target_start = None
        goal_states = getattr(self.planningProblem.goal, "state_list", [])
        for gs in goal_states:
            if getattr(gs, "position", None) is not None or getattr(gs, "velocity", None) is not None or getattr(gs, "orientation", None) is not None:
                self.time_only_goal = False
            if not self.is_time_dependent and getattr(gs, "time_step", None) is not None:
                self.is_time_dependent = True
                if self.survival_target_start is None:
                    gs_time_step = getattr(gs, "time_step", INT_MAX)
                    self.survival_target_start = int(gs_time_step.end)

        
    def evaluation_function(self, node_current: PriorityNode) -> float:
        ########################################################################
        # todo: Implement your own evaluation function here.                   #
        ########################################################################
        node_current.priority = self.heuristic_function(node_current=node_current)
        return node_current.priority

    def heuristic_function(self, node_current: PriorityNode) -> float:
        ########################################################################
        # todo: Implement your own heuristic cost calculation here.            #
        # Hint:                                                                #
        #   Use the State of the current node and the information from the     #
        #   planning problem, as well as from the scenario.                    #
        #   Some helper functions for your convenience can be found in         #
        #   ./search_algorithms/base_class.py                             #
        ########################################################################

        if self.time_only_goal:
            return self.survival_heuristic_function(node_current=node_current)

        path_last = node_current.list_paths[-1]

        distStartState = self.calc_heuristic_distance(path_last[0])
        distLastState = self.calc_heuristic_distance(path_last[-1])

        if distLastState is None:
            return np.inf

        if distStartState < distLastState:
            return 10000

        cost_lanelet, final_lanelet_id, start_lanelet_id = self.calc_heuristic_lanelet(
            path_last
        )

        if cost_lanelet is None or final_lanelet_id is None or final_lanelet_id[0] is None:
            return np.inf

        efficency = self.calc_path_efficiency(path_last)

        # Resolve time_step to an int (handle Interval-like objects)
        state_time = path_last[-1].time_step
        time_step_int = None
        if isinstance(state_time, int):
            time_step_int = state_time
        elif hasattr(state_time, "end"):
            try:
                time_step_int = int(state_time.end)
            except Exception:
                time_step_int = None
        elif hasattr(state_time, "start"):
            # prefer start of interval
            try:
                time_step_int = int(state_time.start)
            except Exception:
                time_step_int = None


        if time_step_int is None:
            return np.inf

        factor = 1
        numObstaclesInLane = self.num_obstacles_in_lanelet_at_time_step(time_step_int, final_lanelet_id[0])
        isGoalInLane = self.is_goal_in_lane(final_lanelet_id[0])
        if isGoalInLane:
            factor = factor * 0.1
        if self.dict_lanelets_costs[final_lanelet_id[0]] == -1:
            return np.inf
        if start_lanelet_id is None or self.dict_lanelets_costs[final_lanelet_id[0]] > self.dict_lanelets_costs[start_lanelet_id[0]]:
           return np.inf
        if self.dict_lanelets_costs[final_lanelet_id[0]] < self.dict_lanelets_costs[start_lanelet_id[0]]:
          factor = factor * 0.1

        angleToGoal = self.calc_angle_to_goal(path_last[-1])

        orientationToGoalDiff = self.calc_orientation_diff(
            angleToGoal, path_last[-1].orientation
        )
        if final_lanelet_id[0] in self.list_ids_lanelets_goal:
            factor = factor * 0.07
        pathLength = self.calc_travelled_distance(path_last)
        weights = np.zeros(6)
        if distLastState < 0.5:
            factor = factor * 0.00001
        # elif math.pi - abs(abs(laneletOrientationAtPosition - path[-1].orientation)

        if hasattr(self.planningProblem.goal.state_list[0], "velocity"):
            v_mean_goal = (
                self.planningProblem.goal.state_list[0].velocity.start
                + self.planningProblem.goal.state_list[0].velocity.end
            ) / 2
            dist_vel = abs(path_last[-1].velocity - v_mean_goal)
        else:
            dist_vel = 0

        weights[0] = 8.7
        weights[1] = 0.1
        weights[2] = 5
        weights[3] = 1
        weights[4] = 0.005
        weights[5] = 1
        cost = (
            weights[0] * (cost_lanelet / len(path_last))
            + weights[1] * abs(orientationToGoalDiff)
            + weights[2] * distLastState
            + weights[3] * efficency
            + weights[4] * (100 - pathLength)
            + weights[5] * dist_vel
        )

        if cost < 0:
            cost = 0
        return cost * factor


    def survival_heuristic_function(self, node_current: PriorityNode) -> float:
        """
        Survival heuristic: survive until survival_target_start time step.
        
        Cost components:
        - Time progression: encourage advancing toward the survival time goal
        - Acceleration: penalize high accelerations for comfort
        - Steering angle: penalize large steering angles for stability
        - Steering rate: penalize rapid steering changes for smoothness
        - Lane center offset: penalize deviation from reference path
        """
        # If no survival target was extracted, we cannot evaluate this heuristic reliably
        if self.survival_target_start is None:
            raise ValueError("Survival target start time is not defined for time-only goal.")

        path_last = node_current.list_paths[-1]

        # 1. Time progression: lower cost for nodes closer to survival target
        time_remaining = self.survival_target_start - node_current.depth_tree

        # 2. Acceleration cost: sum of |a| for comfort
        accel_cost = self.calc_acceleration_cost(path_last)

        # 3. Steering angle cost: sum of |delta| for stability
        steer_angle_cost = self.calc_steering_angle_cost(path_last)

        # 4. Steering rate cost: sum of |delta_dot| for smoothness
        steer_rate_cost = self.calc_steering_velocity_cost(path_last)

        # 5. Lane center offset: sum of distances to reference path
        lane_offset_cost = self.calc_distance_to_reference_path(node_current)

        # Weights: balance time convergence with comfort and lane-keeping
        # [time_progress, acceleration, steering_angle, steering_rate, lane_offset]
        w_time = 100.0
        w_accel = 1.0
        w_steer_angle = 0.5
        w_steer_rate = 0.3
        w_lane_offset = 10.0

        cost = (
            w_time * time_remaining
            + w_accel * accel_cost
            + w_steer_angle * steer_angle_cost
            + w_steer_rate * steer_rate_cost
            + w_lane_offset * lane_offset_cost
        )

        return max(cost, 0.0)
    
    def calc_distance_to_reference_path(self, node_current: PriorityNode) -> float:
        """
        Calculate sum of distances from each point on the last path to the
        nearest point on the reference path polyline using
        `calc_distance_to_nearest_point` for segment-aware distances.
        """

        path_last = node_current.list_paths[-1]
        # Ensure reference coordinates are a (N,2) array
        ref_coords = np.asarray(self.reference_path_coords).reshape(-1, 2)

        total_distance = 0.0
        for state in path_last:
            # Extract numeric position (handle Shapes with .center or sequence-like positions)
            if hasattr(state.position, "center"):
                pos = np.array(state.position.center)
            else:
                pos = np.asarray(state.position, dtype=float)
            # Use helper from SearchBaseClass which computes distance to polyline
            dist = abs(self.calc_distance_to_nearest_point(ref_coords, pos))
            total_distance += dist

        return total_distance