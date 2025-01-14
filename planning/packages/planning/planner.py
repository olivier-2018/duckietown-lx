from typing import List
import math

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    PlacedPrimitive,
    PlanningQuery,
    PlanningResult,
    PlanningSetup,
    PlanStep,
    Circle,
    Rectangle,
    SimulationResult,
    simulate,
)

__all__ = ["Planner"]


class Planner:
    params: PlanningSetup

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: PlanningSetup):
        context.info("initialized")
        self.params = data

        # This is the interval of allowed linear velocity
        # Note that min_velocity_x_m_s and max_velocity_x_m_s might be different.
        # Note that min_velocity_x_m_s may be 0 in advanced exercises (cannot go backward)
        max_velocity_x_m_s: float = self.params.max_linear_velocity_m_s
        min_velocity_x_m_s: float = self.params.min_linear_velocity_m_s

        # This is the max curvature. In earlier exercises, this is +inf: you can turn in place.
        # In advanced exercises, this is less than infinity: you cannot turn in place.
        max_curvature: float = self.params.max_curvature

        # these have the same meaning as the collision exercises
        body: List[PlacedPrimitive] = self.params.body
        environment: List[PlacedPrimitive] = self.params.environment

        # these are the final tolerances - the precision at which you need to arrive at the goal
        tolerance_theta_deg: float = self.params.tolerance_theta_deg
        tolerance_xy_m: float = self.params.tolerance_xy_m

        # For convenience, this is the rectangle that contains all the available environment,
        # so you don't need to compute it
        bounds: Rectangle = self.params.bounds

    def on_received_query(self, context: Context, data: PlanningQuery):
        # A planning query is a pair of initial and goal poses
        start: FriendlyPose = data.start
        goal: FriendlyPose = data.target

        # You start at the start pose. You must reach the goal with a tolerance given by
        # tolerance_xy_m and tolerance_theta_deg.

        # You need to declare if it is feasible or not
        feasible = True

        if not feasible:
            # If it's not feasible, just return this.
            result: PlanningResult = PlanningResult(False, None)
            context.write("response", result)
            return

        # If it is feasible you need to provide a plan.

        # Challenge: lx22-planning-dd-empty-vali
        plan: List[PlanStep] = []

        # A plan step consists in a duration, a linear and angular velocity.

        # First rotate towards the goal
        angle_to_goal = math.atan2(goal.y - start.y, goal.x - start.x) * 180 / math.pi
        if angle_to_goal != 0.0:
            duration_turn_deg_s = angle_to_goal / self.params.max_angular_velocity_deg_s
            rot_dir = 1.0
            if angle_to_goal < 0: rot_dir = -1.0
            turn_to_goal = PlanStep(duration = rot_dir * duration_turn_deg_s,
                                    angular_velocity_deg_s = rot_dir * self.params.max_angular_velocity_deg_s,
                                    velocity_x_m_s=0.0)
            plan.append(turn_to_goal)

        # Second, move towards the goal
        dist = ( (goal.y - start.y)**2 + (goal.x - start.x)**2 )**(1/2)
        if dist != 0.0:
            duration_straight_m_s = dist / self.params.max_linear_velocity_m_s
            reach_goal = PlanStep(  duration = duration_straight_m_s,
                                    angular_velocity_deg_s = 0.0,
                                    velocity_x_m_s = self.params.max_linear_velocity_m_s )
            plan.append(reach_goal)
        
        print("=======================")
        print(f"start angle: {start.theta_deg}")
        print(f"goal angle: {goal.theta_deg}")
        print(f"angle_to_goal: {angle_to_goal}")

        # Third, align to goal's angle
        theta_in = angle_to_goal
        if theta_in < 0: theta_in += 360
        theta_out = goal.theta_deg
        delta1 = abs(theta_out - theta_in)
        delta2 = abs(360+theta_out - theta_in)
        delta = min(delta1,delta2)
        shift = 0.0
        if delta == delta2 : shift += 360
        rot_dir = (((abs(theta_in)-abs(theta_out+shift)) < 0) * 1 - 0.5 ) *2
        print(f"delta: {delta} rot_dir:{rot_dir}")

        if delta != 0.0:
            duration_turn_deg_s = delta / self.params.max_angular_velocity_deg_s
            velocity = rot_dir * self.params.max_angular_velocity_deg_s
            print(f"duration: {duration_turn_deg_s} angular_velocity_deg_s:{velocity}")

            align_to_goal = PlanStep(duration = duration_turn_deg_s ,
                                     angular_velocity_deg_s = velocity ,
                                     velocity_x_m_s = 0.0)
            plan.append(align_to_goal)

        result: PlanningResult = PlanningResult(feasible, plan)
        context.write("response", result)
