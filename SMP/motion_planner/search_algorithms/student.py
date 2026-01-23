import re
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
        node_current.priority = self.heuristic_function(node_current=node_current)
        return node_current.priority

    def heuristic_function(self, node_current: PriorityNode) -> float:
        if self.time_only_goal:
            return self.survival_heuristic_function(node_current=node_current)

        path_last = node_current.list_paths[-1]
        current_state = path_last[-1]

        # Lanelet validity check
        cost_lanelet, final_lanelet_id, _ = self.calc_heuristic_lanelet(path_last)
        if final_lanelet_id is None or final_lanelet_id[0] is None:
            return np.inf

        # === Cost Components ===
        # 1. Optimal Velocity: |current_velocity - optimal_velocity|
        #    optimal_velocity = distance_to_goal / time_remaining
        positional_offset_goal = self.calc_heuristic_distance(current_state)
        time_remaining = self.time_desired.end - current_state.time_step
        if time_remaining > 0:
            optimal_velocity = positional_offset_goal / (time_remaining * self.scenario.dt)
        else:
            optimal_velocity = current_state.velocity  # No penalty if past deadline
        optimal_velocity_cost = abs(current_state.velocity - optimal_velocity)

        # 2. Positional Score: sum of distances from path states to reference path
        positional_score = self.calc_distance_to_reference_path(node_current)

        # 3. Orientational Score: sum of orientation diffs from path states to reference path tangent
        orientational_score = self.calc_orientation_score_to_reference_path(node_current)

        # 4. Positional Offset Goal: distance to goal position (already computed above)
        # positional_offset_goal is already set

        # 5. Orientational Offset Goal: orientation difference to goal orientation
        goal_orientation = self.get_goal_orientation()
        if goal_orientation is not None:
            orientational_offset_goal = self.calc_orientation_diff(current_state.orientation, goal_orientation)
        else:
            orientational_offset_goal = 0.0

        # 6. Velocity Offset: |current_velocity - goal_velocity_mean|
        goal_state = self.planningProblem.goal.state_list[0]
        if hasattr(goal_state, "velocity") and goal_state.velocity is not None:
            v_goal = goal_state.velocity
            v_mean_goal = (v_goal.start + v_goal.end) / 2
            velocity_offset = abs(current_state.velocity - v_mean_goal)
        else:
            velocity_offset = 0.0

        # === Weights ===
        weights = np.zeros(50)
        weights[0] = 100      # optimal_velocity_cost
        weights[1] = 50      # positional_score
        weights[2] = 80      # orientational_score
        weights[3] = 100     # positional_offset_goal
        weights[4] = 10      # orientational_offset_goal
        weights[5] = 10      # velocity_offset

        cost = (
            weights[0] * optimal_velocity_cost
            + weights[1] * positional_score
            + weights[2] * orientational_score
            + weights[3] * positional_offset_goal
            + weights[4] * orientational_offset_goal
            + weights[5] * velocity_offset
        )

        if cost < 0:
            cost = 0
        return cost


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
        w_steer_rate = 20
        w_lane_offset = 30.0

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
                raise NotImplementedError("Shape-based positions not implemented in this heuristic.")
            else:
                pos = np.asarray(state.position, dtype=float)
            # Use helper from SearchBaseClass which computes distance to polyline
            dist = abs(self.calc_distance_to_nearest_point(ref_coords, pos))
            total_distance += dist

        return total_distance

    def calc_orientation_score_to_reference_path(self, node_current: PriorityNode) -> float:
        """
        Calculate sum of orientation differences between each state in the last path
        and the reference path tangent at the closest point.
        """
        path_last = node_current.list_paths[-1]
        ref_coords = np.asarray(self.reference_path_coords).reshape(-1, 2)

        total_diff = 0.0
        for state in path_last:
            pos = np.asarray(state.position, dtype=float)
            # Find closest index on reference path
            idx = self.find_closest_vertex(ref_coords, pos)
            # Get tangent orientation at that index
            ref_orientation = self.get_reference_orientation_at_index(ref_coords, idx)
            total_diff += self.calc_orientation_diff(state.orientation, ref_orientation)

        return total_diff

    def get_reference_orientation_at_index(self, ref_coords: np.ndarray, idx: int) -> float:
        """
        Get the tangent orientation at a given index on the reference path.
        Uses the vector from current point to next point (or previous to current if at end).
        """
        n = len(ref_coords)
        if idx < n - 1:
            dx = ref_coords[idx + 1, 0] - ref_coords[idx, 0]
            dy = ref_coords[idx + 1, 1] - ref_coords[idx, 1]
        else:
            dx = ref_coords[idx, 0] - ref_coords[idx - 1, 0]
            dy = ref_coords[idx, 1] - ref_coords[idx - 1, 1]
        return np.arctan2(dy, dx)

    def get_goal_orientation(self) -> float:
        """
        Get goal orientation. If not available, compute from reference path end tangent.
        Returns None if goal has no orientation and reference path fallback is used.
        """
        goal_state = self.planningProblem.goal.state_list[0]
        if hasattr(goal_state, "orientation") and goal_state.orientation is not None:
            return (goal_state.orientation.start + goal_state.orientation.end) / 2
        # No goal orientation specified - return None to indicate no penalty
        return None