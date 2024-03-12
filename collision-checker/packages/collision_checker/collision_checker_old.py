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

    print(f"@@@@@ no of environment: {len(environment)}")
    for i,env in enumerate(environment):
        if isinstance(env.primitive, Circle): 
            print(f"id,type,x,y,theta,rad={i},'circle',{env.pose.x},{env.pose.y},{env.pose.theta_deg},{env.primitive.radius}")
        else:
            print(f"id,type,x,y,theta,dx,dy={i},'Rectangle',{env.pose.x},{env.pose.y},{env.pose.theta_deg},{env.primitive.xmax-env.primitive.xmin},{env.primitive.ymax-env.primitive.ymin}")

    # TODO you can start by rototranslating the robot_body by the robot_pose
    rototranslated_robot: List[PlacedPrimitive] = []
    for bot in robot_body: 
        print(f"\n====NEW===== initial robot_body: [{bot.pose.x},{bot.pose.y},{bot.pose.theta_deg}] ")
        print(f"====NEW===== robot_pose: [{robot_pose.x},{robot_pose.y},{robot_pose.theta_deg}]") 
        print(f"x,y,theta,dx,dy={robot_pose.x},{robot_pose.y},{robot_pose.theta_deg},{bot.primitive.xmax-bot.primitive.xmin},{bot.primitive.ymax-bot.primitive.ymin}")
        
        bot.pose.x = robot_pose.x
        bot.pose.y = robot_pose.y
        bot.pose.theta_deg = robot_pose.theta_deg
        print(f"====NEW===== final robot_body: [{bot.pose.x},{bot.pose.y},{bot.pose.theta_deg}] ")
        
        rototranslated_robot.append(bot)

    # Then, call check_collision_list to see if the robot collides with the environment
    collided = check_collision_list(rototranslated_robot, environment)
    print(f"ENDEND collided:{collided}")
    print(collided)    

    # TODO return the status of the collision
    if collided:
        return True
    else:
        return False

    # for now let's return a random guess
    #return random.uniform(0, 1) > 0.5


def check_collision_list(rototranslated_robot: List[PlacedPrimitive], 
                         environment: List[PlacedPrimitive]) -> bool:

    # This is just some code to get you started, but you don't have to follow it exactly
    for robot, envObject in itertools.product(rototranslated_robot, environment):
        if check_collision_shape(robot, envObject):
            return True

    return False

def check_circle_to_circle(x1,y1,r1,x2,y2,r2):
    dist_centers = ( (x2 - x1)**2 + (y2 - y1)**2 )**(1/2)
    combined_radii = r1 + r2
    if dist_centers < combined_radii: 
        if DEBUG: print("check_circle_to_circle: COLLISION=True")
        return True
    else: 
        return False

def check_circle_to_rectangle(segments, xc, yc, radius):
    for (p1, p2) in segments:
        # Evaluate circle center projection point on segments as curvilinear percentage  
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        u = ((xc - p1[0]) * dx + (yc - p1[1]) * dy) / float(dx**2 + dy**2)
        # truncate u to segment edges
        if u > 1: u = 1
        elif u < 0: u = 0
        # Evaluate projection point within segment
        x = p1[0] + u * dx
        y = p1[1] + u * dy
        # evaluate distance from circle center to projection point on segment
        dist = ((x - xc)**2 + (y - yc)**2)**(1/2)
        # Decision
        if dist < radius:
            if DEBUG: print("check_circle_to_rectangle: COLLISION=True")
            return True
    # If no intersection with any segment then return False
    if DEBUG: print("check_circle_to_rectangle: no COLLISION")
    return False


def check_collision_shape(a: PlacedPrimitive, 
                          b: PlacedPrimitive) -> bool:
    # TODO check if the two primitives are colliding
    # TODO return the status of the collision
    # https://stackoverflow.com/questions/24727773/detecting-rectangle-collision-with-a-circle
    # https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
    # https://www.baeldung.com/cs/circle-line-segment-collision-detection
    # https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle

    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        if DEBUG: print("========= circle vs circle")
        return check_circle_to_circle(a.pose.x, a.pose.y, a.primitive.radius, 
                                      b.pose.x, b.pose.y, b.primitive.radius)

    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        if DEBUG: print("========= Rectangle vs circle")

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
        # debug
        if DEBUG: print(f"xc,yc,radius = {xc},{yc},{radius}")
        if DEBUG: print(f"xr,yr,theta,xpoints, ypoints = {xr},{yr},{theta*180/np.pi},{[p1[0],p2[0],p3[0],p4[0]]}, {[p1[1],p2[1],p3[1],p4[1]]}")
        # create list of segments from rotated rectangle corners
        segments = [(p1,p2), (p2, p3), (p3, p4), (p4,p1)]        

        return check_circle_to_rectangle(segments, xc, yc, radius)


    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        if DEBUG: print("========= Rectangle vs Rectangle")

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
        # debug
        #print(f"R1p1,R1p2,R1p3,R1p4,R2p1,R2p2,R2p3,R2p4 = {tuple(R1p1)},{tuple(R1p2)},{tuple(R1p3)},{tuple(R1p4)},{tuple(R2p1)},{tuple(R2p2)},{tuple(R2p3)},{tuple(R2p4)} #after rotation&trsnlation")
        if DEBUG: print(f"R1x,R1y,R1theta,R1xpts,R1ypts = {R1x},{R1y},{R1theta*180/np.pi},{[R1p1[0],R1p2[0],R1p3[0],R1p4[0]]}, {[R1p1[1],R1p2[1],R1p3[1],R1p4[1]]}")
        if DEBUG: print(f"R2x,R2y,R2theta,R2xpts,R2ypts = {R2x},{R2y},{R2theta*180/np.pi},{[R2p1[0],R2p2[0],R2p3[0],R2p4[0]]}, {[R2p1[1],R2p2[1],R2p3[1],R2p4[1]]}")
        # create list of segments from rotated rectangle corners
        R1segments = [(R1p1,R1p2), (R1p2, R1p3), (R1p3, R1p4), (R1p4,R1p1)] 
        R2segments = [(R2p1,R2p2), (R2p2, R2p3), (R2p3, R2p4), (R2p4,R2p1)]    
        
        # Heuristic checks (lower/upper bounds)
        print("heuristics checks.")
        if DEBUG: 
            print(f"R1x,R1y,R1theta,R1xmin,R1xmax,R1ymin,R1ymax={R1x},{R1y},{R1theta},{R1xmin},{R1xmax},{R1ymin},{R1ymax}")
            print(f"R2x,R2y,R2theta,R2xmin,R2xmax,R2ymin,R2ymax={R2x},{R2y},{R2theta},{R2xmin},{R2xmax},{R2ymin},{R2ymax}")
        # First check if R1C1 collides with R2 shape
        R1C1_radius = min(R1xmax-R1xmin, R1ymax-R1ymin)/2
        if check_circle_to_rectangle(R2segments, R1x, R1y, R1C1_radius):
            if DEBUG: print("heuristics check1: COLLISION !!")
            return True
        # Second check if R1C2 collides with R2 shape
        R1C2_radius = np.linalg.norm(R1p3 - R1p1)/2
        if not check_circle_to_rectangle(R2segments, R1x, R1y, R1C2_radius):
            # Third, check R1 (bot) included in R2 (coarse estimate, does not capture all instances)
            # info: assumes R1 is the bot
            R2C1_radius = min(R2xmax-R2xmin, R2ymax-R2ymin)/2
            dist_centers = ((R2x - R1x)**2 + (R2y - R1y)**2)**(1/2)
            if DEBUG: print(f"### dist_centers:{dist_centers} R2C1_radius:{R2C1_radius} R1C2_radius:{R1C2_radius}")
            if dist_centers+R1C2_radius < R2C1_radius:
                print("heuristics check2: COLLISION")
                return True
            if DEBUG: print("heuristics check3: NO COLLISION")
            return False

        # Full inclusion check () 

        # Full checks for each segment
        if DEBUG: print("Full checks for each segment")
        for S1p1, S1p2 in R1segments:
            if DEBUG: print("segment1:",s1)
            for S2p1, S2p2 in R2segments:
                if DEBUG: print("segment2:",s2)
                # evaluate determinant of segment crossing 
                S1dx = S1p2[0] - S1p1[0]
                S1dy = S1p2[1] - S1p1[1]
                S2dx = S2p2[0] - S2p1[0]
                S2dy = S2p2[1] - S2p1[1]
                R1determinant = ( S2dy*(S2p2[0]-S1p1[0]) - S2dx*(S2p2[1]-S1p1[1]) ) * ( S2dy*(S2p2[0]-S1p2[0]) - S2dx*(S2p2[1]-S1p2[1]) )
                R2determinant = ( S1dy*(S1p2[0]-S2p1[0]) - S1dx*(S1p2[1]-S2p1[1]) ) * ( S1dy*(S1p2[0]-S2p2[0]) - S1dx*(S1p2[1]-S2p2[1]) )
                del S2p1, S2p2 # 
                # decision
                if (R1determinant <= 0) & (R2determinant <= 0):
                    if DEBUG: print("each segment check: COLLISION=True #True")
                    return True
            del S1p1, S1p2
        # If no intersection with any segment then return False
        if DEBUG: print("each segment check: COLLISION=False")
        return False
        
