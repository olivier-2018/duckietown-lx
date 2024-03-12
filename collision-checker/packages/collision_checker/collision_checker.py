import itertools
import random
from typing import List
import numpy as np

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

__all__ = ["CollisionChecker"]

DEBUG = False

class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision( environment=self.params.environment, 
                                    robot_body=self.params.body, 
                                    robot_pose=data.pose)
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(environment: List[PlacedPrimitive], 
                    robot_body: List[PlacedPrimitive], 
                    robot_pose: FriendlyPose) -> bool:

    # TODO you can start by rototranslating the robot_body by the robot_pose
    if DEBUG: print(f"====NEW-ENV=====")
    rototranslated_robot: List[PlacedPrimitive] = []
    for bot in robot_body: 
        bot.pose.x = robot_pose.x
        bot.pose.y = robot_pose.y
        bot.pose.theta_deg = robot_pose.theta_deg
        if DEBUG: print(f"-------- Robot pose: [{bot.pose.x},{bot.pose.y},{bot.pose.theta_deg}] ")
        rototranslated_robot.append(bot)

    # Then, call check_collision_list to see if the robot collides with the environment
    collided = check_collision_list(rototranslated_robot, environment)
    print(f"====ENV-CHECK-END=====  collided:{collided}")

    # TODO return the status of the collision
    return collided
    

def check_collision_list(rototranslated_robot: List[PlacedPrimitive], 
                         environment: List[PlacedPrimitive]) -> bool:

    # This is just some code to get you started, but you don't have to follow it exactly
    for robot, envObject in itertools.product(rototranslated_robot, environment):
        if check_collision_shape(robot, envObject):
            return True
    return False


def check_collision_shape(a: PlacedPrimitive, 
                          b: PlacedPrimitive) -> bool:
    # TODO check if the two primitives are colliding
    # TODO return the status of the collision

    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        if DEBUG: print("-------- circle vs circle")
        dist = ( (x2 - x1)**2 + (y2 - y1)**2 )**(1/2)
        combined_radii = r1 + r2
        if dist < combined_radii: 
            if DEBUG: print("COLLISION detected")
            return True
        else: 
            return False

    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        if DEBUG: print("-------- Rectangle vs circle")

        # extract info so it's understandable
        xmin,ymin = a.primitive.xmin, a.primitive.ymin
        xmax,ymax = a.primitive.xmax, a.primitive.ymax
        xr,yr,theta = a.pose.x, a.pose.y, a.pose.theta_deg * np.pi / 180
        xc,yc,radius = b.pose.x, b.pose.y, b.primitive.radius
        # Create rotation matrix
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        # Create list of points (going CW) and rotate them
        p1 = R @ np.array([xmin, ymin]) + np.array([xr, yr])
        p2 = R @ np.array([xmin, ymax]) + np.array([xr, yr])
        p3 = R @ np.array([xmax, ymax]) + np.array([xr, yr])
        p4 = R @ np.array([xmax, ymin]) + np.array([xr, yr])
        # create list of segments from rotated rectangle corners
        XY_pts = np.stack((p1,p2,p3,p4)) 
        if DEBUG: print(f"xr,yr,theta,R1_XY_pts = {xr},{yr},{theta*180/np.pi},{XY_pts}")
        # check inclusion of R1 points into Circle
        inclusion_flag = []
        for P in XY_pts:
            dist = ((P[0]-xc)**2 + (P[1]-yc)**2)**(1/2)
            if dist < radius: 
                inclusion_flag.append(True)
            else:
                inclusion_flag.append(False)
        if DEBUG: print(f"inclusion_flag: {inclusion_flag}")
        # Decision 
        if set(inclusion_flag)=={True} or len(set(inclusion_flag))>1:
            if DEBUG: print("COLLISION detected")
            return True
        elif set(inclusion_flag)=={False}:
            if DEBUG: print("No COLLISION detected")
            return False


    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        if DEBUG: print("-------- Rectangle vs Rectangle")

        # extract info so it's understandable
        R1xmin,R1ymin = a.primitive.xmin, a.primitive.ymin
        R1xmax,R1ymax = a.primitive.xmax, a.primitive.ymax
        R1x,R1y,R1theta = a.pose.x, a.pose.y, a.pose.theta_deg * np.pi / 180
        R2xmin,R2ymin = b.primitive.xmin, b.primitive.ymin
        R2xmax,R2ymax = b.primitive.xmax, b.primitive.ymax
        R2x,R2y,R2theta = b.pose.x, b.pose.y, b.pose.theta_deg * np.pi / 180

        # Create rotation matrix
        R1rot = np.array([[np.cos(R1theta), -np.sin(R1theta)],
                          [np.sin(R1theta), np.cos(R1theta)]])
        R2rot = np.array([[np.cos(R2theta), -np.sin(R2theta)],
                          [np.sin(R2theta), np.cos(R2theta)]])
        # Create list of points (going CW) and rotate them
        R1p1 = R1rot @ np.array([R1xmin, R1ymin]) + np.array([R1x, R1y])
        R1p2 = R1rot @ np.array([R1xmin, R1ymax]) + np.array([R1x, R1y])
        R1p3 = R1rot @ np.array([R1xmax, R1ymax]) + np.array([R1x, R1y])
        R1p4 = R1rot @ np.array([R1xmax, R1ymin]) + np.array([R1x, R1y])
        R2p1 = R2rot @ np.array([R2xmin, R2ymin]) + np.array([R2x, R2y])
        R2p2 = R2rot @ np.array([R2xmin, R2ymax]) + np.array([R2x, R2y])
        R2p3 = R2rot @ np.array([R2xmax, R2ymax]) + np.array([R2x, R2y])
        R2p4 = R2rot @ np.array([R2xmax, R2ymin]) + np.array([R2x, R2y])
        # Create polygon
        R1_XY_pts = np.stack((R1p1,R1p2,R1p3,R1p4))
        R2_XY_pts = np.stack((R2p1,R2p2,R2p3,R2p4))
        # check inclusion of R1 points into R2
        inclusion_flag = []
        for P in R1_XY_pts:
            Vp = np.array([P-R2_XY_pts[0]])[0]
            V1 = np.array([R2_XY_pts[1]-R2_XY_pts[0]])[0]
            V2 = np.array([R2_XY_pts[-1]-R2_XY_pts[0]])[0]
            if (0<Vp.dot(V1)<V1.dot(V1) and 0<Vp.dot(V2)<V2.dot(V2)): 
                inclusion_flag.append(True)
            else:
                inclusion_flag.append(False)
        if DEBUG: print(f"inclusion_flag: {inclusion_flag}")
        # Decision 
        if set(inclusion_flag)=={True} or len(set(inclusion_flag))>1:
            if DEBUG: print("COLLISION detected")
            return True
        elif set(inclusion_flag)=={False}:
            if DEBUG: print("No COLLISION detected")
            return False
        
