import math

from sympy import Point, Line
from sympy.geometry import Segment
from sympy.geometry.util import intersection


# A Python3 program to find if 2 given line segments intersect or not

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def slope(p, q):  # Line slope given two points:
    return (q.y - p.y) / (q.x - p.x)


def angle(p1, q1, p2, q2):
    s1 = slope(p1, q1)
    s2 = slope(p2, q2)
    return math.degrees(math.atan((s2 - s1) / (1 + (s2 * s1))))


# Given three collinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
def onSegment(p, q, r):
    if ((q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
            (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False


def orientation(p, q, r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise

    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    # for details of below formula.

    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    if (val > 0):

        # Clockwise orientation
        return 1
    elif (val < 0):

        # Counterclockwise orientation
        return 2
    else:

        # Collinear orientation
        return 0


# The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1, q1, p2, q2):
    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True

    # Special Cases

    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True

    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True

    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True

    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True

    # If none of the cases
    return False


def gen_crosswalk_route(routes, crosswalk_info):
    # find a straight route crossing a crosswalk
    start_waypoint = None
    end_waypoint = None

    for route in routes:
        meet_crosswalk, intersection_crosswalk = get_crosswalk_info(route.start_waypoint, route.end_waypoint,
                                                                    crosswalk_info)
        # print(road.meet_crosswalk)
        if meet_crosswalk and route.direction == 0:
            # find the predecessor_route to get the start waypoint
            predecessor_route = get_predecessor_route(route.start_waypoint, routes)
            if predecessor_route:
                # identify the end waypoint
                end_waypoint = route.end_waypoint
                start_waypoint = predecessor_route.start_waypoint
                cross_waypoint = route.start_waypoint
                # calculate the waypoint behind the crosswalk

                break

    print(start_waypoint, end_waypoint, cross_waypoint)
    return start_waypoint, end_waypoint, cross_waypoint


def get_crosswalk_routes(routes, crosswalk_info, direction, num=1):
    target_routes = []
    for route in routes:
        meet_crosswalk, intersection_crosswalk = get_crosswalk_info(route.start_waypoint, route.end_waypoint,
                                                                    crosswalk_info)
        if meet_crosswalk and route.direction == direction:
            target_routes.append(route)

        if len(target_routes) == num:
            break

    return target_routes


def find_predecessor_lanes(map_info, query_lane):
    predecessor_lanes = []
    for road in map_info.roads:
        if road.id == query_lane[0]:
            if road.link.predecessor.elementType == 'road':
                road_id = road.link.predecessor.elementId

                for lane in road.lanes.laneSections[0].allLanes:
                    if lane.id == query_lane[1]:
                        lane_id = lane.link.predecessorId
                        predecessor_lanes.append((road_id, lane_id))
                        break

                break

    return predecessor_lanes


def find_predecessor_route(routes, query_lane):
    predecessor_route = []
    for route in routes:
        if route.end_waypoint.road_id == query_lane[0] and route.end_waypoint.lane_id == query_lane[1]:
            predecessor_route.append(route)

    return predecessor_route


def find_predecessor_lanes(map_info, query_lane):
    predecessor_lanes = []
    for road in map_info.roads:
        if road.id == query_lane[0]:
            if road.link.predecessor.elementType == 'road':
                road_id = road.link.predecessor.elementId

                for lane in road.lanes.laneSections[0].allLanes:
                    if lane.id == query_lane[1]:
                        lane_id = lane.link.predecessorId
                        predecessor_lanes.append((road_id, lane_id))
                        break

                break

    return predecessor_lanes


def get_crosswalk_routes(routes, crosswalk_info, direction, num=1):
    target_routes = []
    for route in routes:
        meet_crosswalk, intersection_crosswalk = get_crosswalk_info(route.start_waypoint, route.end_waypoint,
                                                                    crosswalk_info)
        if meet_crosswalk and route.direction == direction:
            target_routes.append(route)

        if len(target_routes) == num:
            break

    return target_routes


def get_route_direction(start_waypoint, end_waypoint):
    if abs(start_waypoint.transform.location.x - end_waypoint.transform.location.x) <= 1 or \
            abs(start_waypoint.transform.location.y - end_waypoint.transform.location.y) <= 1:
        direction = 0
    else:
        # start_waypoint_temp_yaw = start_waypoint
        # end_waypoint_temp = end_waypoint
        start_point_yaw = start_waypoint.transform.rotation.yaw % 360
        end_point_yaw = end_waypoint.transform.rotation.yaw % 360

        if 360 - abs(start_point_yaw) <= 1:
            start_point_yaw = 0

        if 360 - abs(end_point_yaw) <= 1:
            end_point_yaw = 0

        diff = end_point_yaw - start_point_yaw

        if abs(diff) > 180:
            if diff > 0:
                cof = -1
            else:
                cof = 1
            diff = cof * (360 - abs(diff))

        if diff > 0 - 2 and diff < 180 - 2:
            direction = -1  # right
        elif (diff >= 180 - 2 and diff <= 180 + 2) or (diff >= -180 - 2 and diff <= -180 + 2):
            direction = 2  # opposite
        elif diff < 0 - 2 and diff > -180 + 2:
            direction = 1  # left

        else:
            direction = -999

        # print(diff, direction)

    return direction


def get_successor(end_road_id, map_info):
    successor_type = None
    successor_id = None
    for road in map_info.roads:
        if road.id == end_road_id:
            successor_type = road.link.successor.elementType
            successor_id = road.link.successor.elementId
            break
    return (successor_type, successor_id)


def get_predecessor(start_road_id, map_info):
    predecessor_type = None
    predecessor_id = None
    for road in map_info.roads:
        if road.id == start_road_id:
            predecessor_type = road.link.predecessor.elementType
            predecessor_id = road.link.predecessor.elementId
            break
    return (predecessor_type, predecessor_id)


def get_predecessor_route(waypoint, routes):
    target_route = None
    for route in routes:
        if route.end_waypoint.transform.location.x == waypoint.transform.location.x and \
                route.end_waypoint.transform.location.y == waypoint.transform.location.y and \
                route.end_waypoint.transform.location.z == waypoint.transform.location.z:
            target_route = route
            break

    return target_route


def get_waypoint_by_start_road(routes, id):
    waypoint = None
    for route in routes:
        if route.start_route == id:
            waypoint = route.start_waypoint
            break

    return waypoint


def get_crosswalk_info(start_waypoint, end_waypoint, crosswalk_info):
    p1, p2 = map(Point, [(start_waypoint.transform.location.x, start_waypoint.transform.location.y),
                         (end_waypoint.transform.location.x, end_waypoint.transform.location.y)])
    line = Segment(p1, p2)
    crosswalk_intersection = False
    intersection_crosswalk = None
    for i in range(len(crosswalk_info[0])):
        crosswalk = crosswalk_info[0][i]
        waypoints = crosswalk_info[1][i]
        if crosswalk.intersection(line):
            crosswalk_intersection = True
            intersection_crosswalk = waypoints
            intersection(crosswalk, line)
            break

    return crosswalk_intersection, intersection_crosswalk


def ccw(A, B, C):
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)


def intersect(A, B, C, D):
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def get_roadside(map_info, current_road, current_lane):
    road_info = map_info.get_road(current_road)

    # Return true if line segments AB and CD intersect


class Route():
    def __init__(self, id, start_waypoint, end_waypoint, map_info):
        self.id = id
        self.start_waypoint = start_waypoint
        self.end_waypoint = end_waypoint
        self.start_road = start_waypoint.road_id
        self.start_lane = start_waypoint.lane_id
        self.end_road = end_waypoint.road_id
        self.end_lane = end_waypoint.lane_id
        self.length = math.sqrt((self.start_waypoint.transform.location.x - self.end_waypoint.transform.location.x) ** 2
                                + (
                                        self.start_waypoint.transform.location.y - self.end_waypoint.transform.location.y) ** 2)
        self.direction = get_route_direction(start_waypoint, end_waypoint)
        self.successor_road = get_successor(self.end_road, map_info)
        self.predecessor_road = get_predecessor(self.start_road, map_info)
        if self.start_road != self.end_road:
            self.cross_road = True
        else:
            self.cross_road = False
        self.stop_signs = []
        self.signals = []
        self.has_stop_sign, self.has_traffic_light = self.exist_signal(map_info)
        self.has_crosswalk = self.exist_crosswalk(map_info)
        self.has_speed_sign = self.exist_speed_sign(map_info)
        self.num_lanes = self.get_num_lanes(map_info)
        self.road_mark_left, self.road_mark_right = self.get_road_mark()
        self.one_way = self.is_one_way(map_info)
        self.do_not_enter = self.is_do_not_enter(map_info)


    def is_do_not_enter(self, map_info):
        opendrive_road = map_info.get_road(self.start_road)
        result = False

        for obj in opendrive_road.objects:
            if 'donotenter' in obj['name'].lower() and self.start_lane == -3:
                result = True
                break

        return result

    def is_no_turn(self, map_info, routes):
        opendrive_road = map_info.get_road(self.start_road)
        result = None
        for obj in opendrive_road.objects:
            if 'noturnleft' in obj['name'].lower() and self.start_lane == -3 and self.is_intersection(routes):
                print(self.start_lane)
                result = 'no_turn_left_sign'
                break
            elif 'noturnright' in obj['name'].lower() and self.start_lane == 1:
                result = 'no_turn_right_sign'
                break
            elif 'noturn' in obj['name'].lower():
                result = 'no_turn_sign'
                break

        return result

    def is_t_intersection(self, routes):
        # if the route is straight, which means it is a through street, not at an end street
        if self.direction == 0:
            return False, None

        for route in routes:
            intersect, intersect_angle = self.intersect_with(route)
            if route.id != self.id and intersect and intersect_angle > 85 and intersect_angle < 100:
                print('hello', intersect_angle)
                flag = False
                for another_route in routes:
                    # if another route starts from the same waypoint and the direction is 0
                    if abs(another_route.start_waypoint.transform.location.x - self.start_waypoint.transform.location.x) <= 1 and \
                            abs(another_route.start_waypoint.transform.location.y - self.start_waypoint.transform.location.y) <= 1 and \
                            another_route.direction == 0:
                        flag = True
                        break
                if not flag:
                    return True, route

        return False, None

    def is_intersection(self, routes):
        if self.direction != 0:
            return False, None

        for route in routes:
            intersect, intersect_angle = self.intersect_with(route)
            if intersect and intersect_angle > 80 and intersect_angle < 100 and route.direction == 0:
                print(intersect_angle)
                return True, route

        return False, None

    def is_one_way(self, map_info):
        opendrive_road = map_info.get_road(self.start_road)
        if self.start_lane > 0:
            opposite_lanes = opendrive_road.lanes.laneSections[0].rightLanes

        else:
            opposite_lanes = opendrive_road.lanes.laneSections[0].leftLanes

        if len(opposite_lanes) == 0:
            is_one_way = True
        else:
            is_one_way = False

        return is_one_way

    def get_road_mark(self):
        return self.start_waypoint.left_lane_marking.type, self.start_waypoint.right_lane_marking.type

    def get_num_lanes(self, map_info):
        opendrive_road = map_info.get_road(self.start_road)
        if self.start_lane > 0:
            lanes = opendrive_road.lanes.laneSections[0].leftLanes
        else:
            lanes = opendrive_road.lanes.laneSections[0].rightLanes

        num_lanes = 0
        for lane in lanes:
            if lane.type == 'driving':
                num_lanes += 1

        return num_lanes

    def exist_speed_sign(self, map_info):
        result = False
        opendrive_road = map_info.get_road(self.start_road)
        for obj in opendrive_road.objects:
            if obj['name'] == 'speed_sign':
                if self.start_lane in obj['validity_lanes']:
                    result = True
                    break

        return result

    def exist_crosswalk(self, map_info):
        result = False
        opendrive_road = map_info.get_road(self.start_road)
        for obj in opendrive_road.objects:
            if obj['name'] == 'crosswalk' and self.start_lane in obj['validity_lanes']:
                result = True
                break

        return result

    def exist_signal(self, map_info):
        result_stop = False
        result_light = False
        for road in map_info.roads:
            if road.id == self.start_waypoint.road_id:
                for obj in road.signals:
                    if obj['name'] == 'Sign_Stop':
                        if self.start_waypoint.lane_id in obj['validity_lanes']:
                            result_stop = True
                            self.stop_signs.append(obj['id'])

                    elif obj['name'] == 'Signal_3Light_Post01':
                        if self.start_waypoint.lane_id in obj['validity_lanes']:
                            result_light = True
                            self.signals.append(obj['id'])
                break

        return result_stop, result_light

    def intersect_with(self, other_route):
        A = self.start_waypoint.transform.location
        B = self.end_waypoint.transform.location
        C = other_route.start_waypoint.transform.location
        D = other_route.end_waypoint.transform.location

        p1 = Point(A.x, A.y)
        q1 = Point(B.x, B.y)
        p2 = Point(C.x, C.y)
        q2 = Point(D.x, D.y)

        if doIntersect(p1, q1, p2, q2):
            intersect_angle = angle(p1, q1, p2, q2)
            return True, intersect_angle
        else:
            return False, None
        # if intersect(A, B, C, D):
        #     return True
        # else:
        #     return False

    def get_cross_routes(self, routes, direction=0, position=0, num=1):
        # direction: of the route
        # position: related to the ego_vehicle
        target_routes = []
        for route in routes:
            if position == 1:
                relative_position = -1
            elif position == -1:
                relative_position = 1
            else:
                relative_position = position

            intersect, _ = self.intersect_with(route)
            if route.id != self.id and intersect and route.start_waypoint.transform.location != self.end_waypoint.transform.location and \
                    route.direction == direction and \
                    get_route_direction(self.start_waypoint, route.start_waypoint) == relative_position:
                target_routes.append(route)

            if len(target_routes) == num:
                break

        return target_routes

    def get_predecessor_route(self, routes):
        target_route = None
        for route in routes:
            if route.end_waypoint.transform.location.x == self.start_waypoint.transform.location.x and \
                    route.end_waypoint.transform.location.y == self.start_waypoint.transform.location.y and \
                    route.end_waypoint.transform.location.z == self.start_waypoint.transform.location.z:
                target_route = route
                break
        return target_route

    def get_successor_route(self, routes):
        target_route = None
        for route in routes:
            if abs(route.start_waypoint.transform.location.x - self.end_waypoint.transform.location.x) <= 2 and \
               abs(route.start_waypoint.transform.location.y - self.end_waypoint.transform.location.y) <= 2 and \
               abs(route.start_waypoint.transform.location.z - self.end_waypoint.transform.location.z) <= 2:
                target_route = route
                break
        return target_route