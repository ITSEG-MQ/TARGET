import argparse
from lxml import etree
import xml.etree.cElementTree as ET
import pickle
from road_topology import Route
from scenario_runner import ScenarioRunner
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, RunningStopTest, RunningRedLightTest, \
    RouteCompletionTest, ActorSpeedAboveThresholdTest, KeepLaneTest, DecelerateTest, KeepDistanceTest, KeepClearTest, \
    NoTurnTest, GiveWayTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, \
    InTriggerDistanceToLocation, InTimeToArrivalToLocation, InTriggerDistanceToLocation
from srunner.scenariomanager.timer import TimeOut
from srunner.scenariomanager.weather_sim import Weather
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
import yaml
import carla
from opendriveparser import parse_opendrive
from srunner.tools.scenario_parser import ScenarioConfigurationParser
from srunner.tools.route_manipulation import *
from agents.navigation.local_planner import RoadOption
# from manual_control import visualize_scenario
from multiprocessing import Process

DIRECTION = {
    'left': 1,
    'right': -1,
    'straight': 0,
    'opposite': 2,
    'static': 0,
    'go forward': 0,
    'turn left': 1,
    'turn right': -1,
    'change lane to left': 0,
    'change lane to right': 0,
    'decelerate': 0,
}


def create_specific_route(routes, start_road, start_lane, end_road, end_lane):
    start_point = None
    end_point = None
    for route in routes:
        if route.start_road == start_road and route.start_lane == start_lane:
            start_point = route.start_waypoint
        elif route.end_road == end_road and route.end_lane == end_lane:
            end_point = route.end_waypoint.next(5)[0]

        if start_point and end_point:
            break

    if start_point and end_point:
        return start_point, end_point
    else:
        return None, None


class SrunnerArgs():
    def __init__(self, args):
        self.version = args['version']
        self.host = args['host']
        self.port = args['port']
        self.timeout = args['timeout']
        self.trafficManagerPort = 8000
        self.trafficManagerSeed = 0
        self.sync = args['sync']
        self.list = args['list']
        self.scenario = args['scenario']
        self.openscenario = False
        self.route = False
        self.output = True
        self.reloadWorld = args['reloadWorld']
        self.record = ''
        self.configFile = args['configFile']
        self.additionalScenario = ''
        self.outputDir = ''
        self.junit = False
        self.json = False
        # self.file = args['file']
        self.file = None
        self.repetitions = 1
        self.debug = False
        self.waitForEgo = args['waitForEgo']
        self.randomize = False
        self.agent = args['agent']
        self.agentConfig = args['agentConfig']


def load_config(config_file):
    with open(config_file, "r") as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return None

    return config


def get_weather_info(scenario_config):
    return None


def gen_scenario_config(scenario_name, routes):
    scenario_configuration = ScenarioConfigurationParser.parse_scenario_configuration(scenario_name,
                                                                                      scenario_name + '.xml')[0]
    # add ego_vehicle route
    waypoint_list = []
    for i, route in enumerate(routes):
        if i == 0:
            waypoint_list.append(route.start_waypoint.transform.location)
        waypoint_list.append(route.end_waypoint.transform.location)
    scenario_configuration.trajectory = waypoint_list
    return scenario_configuration


def gen_scenario(scenario_configuration, scenario_name, world, ego_vehicles, behavior_config, actor_info,
                 route_info=None,timeout=60):
    scenario = ParsedScenario(scenario_name,
                              ego_vehicles,
                              scenario_configuration,
                              behavior_config,
                              actor_info,
                              world, route_info=route_info,
                              timeout=int(timeout))
    

    return scenario


def get_roadside(route, map_info, direction='left', world=None):
    current_road = route.start_road
    current_lane = route.start_lane
    road_info = map_info.get_road(current_road)
    crosswalk_waypoint = route.start_waypoint.next(5)[0]
    if abs(crosswalk_waypoint.transform.location.x - route.start_waypoint.transform.location.x) > abs(
            crosswalk_waypoint.transform.location.y - route.start_waypoint.transform.location.y):
        crosswalk_direction = 'y'

    else:
        crosswalk_direction = 'x'

    left_offset = 0
    right_offset = 0
    for lane in road_info.lanes.laneSections[0].allLanes:
        if lane.id == current_lane:
            left_offset += lane.widths[0].a / 2
            right_offset += lane.widths[0].a / 2
        elif lane.id > 0 and current_lane <= 0:
            left_offset += lane.widths[0].a
        elif lane.id < 0 and current_lane >= 0:
            right_offset += lane.widths[0].a
        elif current_lane > 0 and lane.id > current_lane:
            left_offset += lane.widths[0].a
        elif current_lane < 0 and lane.id < current_lane:
            right_offset += lane.widths[0].a

    if crosswalk_direction == 'y' and crosswalk_waypoint.transform.location.x > route.start_waypoint.transform.location.x:
        if current_lane > 0:
            right_side_location = carla.Location(crosswalk_waypoint.transform.location.x,
                                                 crosswalk_waypoint.transform.location.y + 2 * right_offset + left_offset,
                                                 crosswalk_waypoint.transform.location.z + 2)
            left_side_location = carla.Location(crosswalk_waypoint.transform.location.x,
                                                crosswalk_waypoint.transform.location.y - left_offset,
                                                crosswalk_waypoint.transform.location.z + 2)
        else:
            right_side_location = carla.Location(crosswalk_waypoint.transform.location.x,
                                                 crosswalk_waypoint.transform.location.y + right_offset,
                                                 crosswalk_waypoint.transform.location.z + 2)
            left_side_location = carla.Location(crosswalk_waypoint.transform.location.x,
                                                crosswalk_waypoint.transform.location.y - 2 * left_offset - right_offset,
                                                crosswalk_waypoint.transform.location.z + 2)
    elif crosswalk_direction == 'y' and crosswalk_waypoint.transform.location.x < route.start_waypoint.transform.location.x:
        if current_lane > 0:
            right_side_location = carla.Location(crosswalk_waypoint.transform.location.x,
                                                 crosswalk_waypoint.transform.location.y - 2 * right_offset - left_offset,
                                                 crosswalk_waypoint.transform.location.z + 2)
            left_side_location = carla.Location(crosswalk_waypoint.transform.location.x,
                                                crosswalk_waypoint.transform.location.y + left_offset,
                                                crosswalk_waypoint.transform.location.z + 2)
        else:
            right_side_location = carla.Location(crosswalk_waypoint.transform.location.x,
                                                 crosswalk_waypoint.transform.location.y - right_offset,
                                                 crosswalk_waypoint.transform.location.z + 2)
            left_side_location = carla.Location(crosswalk_waypoint.transform.location.x,
                                                crosswalk_waypoint.transform.location.y + 2 * left_offset + right_offset,
                                                crosswalk_waypoint.transform.location.z + 2)
    elif crosswalk_direction == 'x' and crosswalk_waypoint.transform.location.y > route.start_waypoint.transform.location.y:
        if current_lane > 0:
            right_side_location = carla.Location(
                crosswalk_waypoint.transform.location.x - 2 * right_offset - left_offset,
                crosswalk_waypoint.transform.location.y,
                crosswalk_waypoint.transform.location.z + 2)
            left_side_location = carla.Location(crosswalk_waypoint.transform.location.x + left_offset,
                                                crosswalk_waypoint.transform.location.y,
                                                crosswalk_waypoint.transform.location.z + 2)
        else:
            right_side_location = carla.Location(crosswalk_waypoint.transform.location.x - right_offset,
                                                 crosswalk_waypoint.transform.location.y,
                                                 crosswalk_waypoint.transform.location.z + 2)
            left_side_location = carla.Location(
                crosswalk_waypoint.transform.location.x + 2 * left_offset + right_offset,
                crosswalk_waypoint.transform.location.y,
                crosswalk_waypoint.transform.location.z + 2)
    elif crosswalk_direction == 'x' and crosswalk_waypoint.transform.location.y < route.start_waypoint.transform.location.y:
        if current_lane > 0:
            right_side_location = carla.Location(
                crosswalk_waypoint.transform.location.x + 2 * right_offset + left_offset,
                crosswalk_waypoint.transform.location.y,
                crosswalk_waypoint.transform.location.z + 2)
            left_side_location = carla.Location(crosswalk_waypoint.transform.location.x - left_offset,
                                                crosswalk_waypoint.transform.location.y,
                                                crosswalk_waypoint.transform.location.z + 2)
        else:
            right_side_location = carla.Location(crosswalk_waypoint.transform.location.x + right_offset,
                                                 crosswalk_waypoint.transform.location.y,
                                                 crosswalk_waypoint.transform.location.z + 2)
            left_side_location = carla.Location(
                crosswalk_waypoint.transform.location.x - 2 * left_offset - right_offset,
                crosswalk_waypoint.transform.location.y,
                crosswalk_waypoint.transform.location.z + 2)

    current_map = world.get_map()
    if direction == 'static':
        waypoint_start = current_map.get_waypoint(right_side_location)
        waypoint_end = current_map.get_waypoint(right_side_location)
    elif direction == 'left_to_right':
            # left to right
        # rotation = carla.Rotation(yaw=route.start_waypoint.transform.rotation.yaw + 90)
        waypoint_start = current_map.get_waypoint(left_side_location)
        waypoint_end = current_map.get_waypoint(right_side_location)

        # waypoint_start = carla.Transform(left_side_location, rotation)
        # waypoint_end = carla.Transform(right_side_location, rotation)
    else:
        # rotation = carla.Rotation(yaw=route.start_waypoint.transform.rotation.yaw - 90)
        # waypoint_start = carla.Transform(right_side_location, rotation)
        # waypoint_end = carla.Transform(left_side_location, rotation)
        waypoint_start = current_map.get_waypoint(right_side_location)
        waypoint_end = current_map.get_waypoint(left_side_location)

    return waypoint_start, waypoint_end



def get_scenario_info(world, client, town, scenario_config, reload_world=1):
    print(town)
    if reload_world == 1:
        if scenario_config['Road network']:
            if 'roundabout' == scenario_config['Road network']['Road type']:
                client.load_world('Town03')
            elif 'do not enter sign' == scenario_config['Road network']['Traffic signs'] or 'no turn left sign' == scenario_config['Road network']['Traffic signs']:
                client.load_world('Town06_Opt')
            #     else:
            elif town != '10' and town != 10:
                client.load_world('Town0{}'.format(town))
            else:
                client.load_world('Town10HD_Opt')
        else:
            if town != '10' and town != 10:
                client.load_world('Town0{}'.format(town))
            else:
                client.load_world('Town10HD_Opt')

    current_map = world.get_map()
    print(current_map)
    CarlaDataProvider.set_world(world)
    fh = open("maps/{}.xodr".format(current_map.name.split('/')[-1]), 'r')
    map_info = parse_opendrive(etree.parse(fh).getroot())
    weather_info = get_weather_info(scenario_config)
    # landmarks = current_map.get_all_landmarks()
    # for landmark in landmarks:
    #     print(landmark.name)
    # get routes
    topology = current_map.get_topology()
    routes = []
    for i, t in enumerate(topology):
        route = Route(i, t[0], t[1], map_info)
        routes.append(route)

    # for route in routes:
    # # if route.has_crosswalk:
    #     draw_route(world, route)
    #     print(route.start_waypoint.transform.location)
    # waypoint_start, waypoint_end, = get_roadside(route, map_info, 'left')
    # world.debug.draw_line(waypoint_start.location, waypoint_end.location,
    #                       life_time=120)

    actor_info = identify_actor_info(routes, scenario_config, map_info, world)
    print('actor_info', actor_info)
    return weather_info, actor_info, current_map

def run(scenario_yaml, model=None, reload_world=1, mode='single', current_town=None):
    scenario_config = load_config(scenario_yaml)

    # p.join()

    print(scenario_yaml)
    # identify actor info
    client = carla.Client(host='127.0.0.1', port=2000)
    world = client.get_world()

    if mode == 'random':
        towns = [1, 2, 3, 4, 5, 6, 7, 10]
        towns = random.sample(towns, len(towns))
        for town in towns:
            print(town)
            weather_info, actor_info, current_map = get_scenario_info(world, client, town, scenario_config, reload_world)
            route_info = {'traffic_light': None, 'stop_sign': None}
            print(actor_info)
            if actor_info and actor_info['ego_vehicle'] and actor_info['ego_vehicle']['route']:
                run_scenario(world, current_map, actor_info, route_info, weather_info, scenario_config, scenario_yaml, model)
                break
            else:
                continue
    elif mode == 'all':
        towns = [1, 2, 3, 4, 5, 6, 7, 10]
        for town in towns:
            print(town)
            weather_info, actor_info, current_map = get_scenario_info(world, client, town, scenario_config, reload_world)
            route_info = {'traffic_light': None, 'stop_sign': None}
            print(actor_info)
            if actor_info and actor_info['ego_vehicle'] and actor_info['ego_vehicle']['route']:
                run_scenario(world, current_map, actor_info, route_info, weather_info, scenario_config, scenario_yaml, model)
                time.sleep(10)
            else:
                continue
    elif mode == 'single':

        print(current_town)
        weather_info, actor_info, current_map = get_scenario_info(world, client, current_town, scenario_config, reload_world)
        route_info = {'traffic_light': None, 'stop_sign': None}
        if actor_info and actor_info['ego_vehicle'] and actor_info['ego_vehicle']['route']:
            # if not os.path.exists('recordings/{}'.format(model)):
            #     os.mkdir('recordings/{}'.format(model))
            # print(scenario_yaml, model, current_town)
            # recording_path = 'recordings/{}/{}-{}'.format(model, scenario_yaml.split('/')[-1].split('.')[0], current_town)
            # if not os.path.exists(recording_path):
            #     os.mkdir(recording_path)
            # p = Process(target=visualize_scenario,
            #             args=(recording_path, ))
            # p.start()
            run_scenario(world, current_map, actor_info, route_info, weather_info, scenario_config, scenario_yaml, model)
            return True
        else:
            print('No test scenario generated')
            return False

def run_scenario(world, current_map, actor_info, route_info, weather_info, scenario_config, scenario_yaml, model=None):
    # result_file = open('result/result_{}.txt'.format(model), 'a')
    destination_file = open('route_destination.txt', 'w')
    for route in actor_info['ego_vehicle']['route']:
        # draw_route(world, route)
        if len(route.stop_signs) != 0:
            # print(route.stop_signs)
            route_stop_signs = []
            for sign_id in route.stop_signs:
                signs = current_map.get_all_landmarks_from_id(sign_id)
                route_stop_signs.append([signs[0].transform, carla.Vector3D(7, 3, 0), sign_id])
            route_info['stop_sign'] = route_stop_signs

            if route.has_traffic_light:
                route_info['traffic_light'] = True

    scenario_name = scenario_yaml.split('/')[1].split('.')[0]
    generate_scenario_xml(current_map.name.split('/')[-1], actor_info, weather_info,
                          scenario_name)
    # generate the destination of the ego-vehicle for ROS driving systems to plan waypoints
    destination_waypoint = actor_info['ego_vehicle']['goal']
    destination_file.write("{} {} {} {}\n".format(destination_waypoint.location.x, destination_waypoint.location.y,
                                                destination_waypoint.location.z, destination_waypoint.rotation.yaw))

    destination_file.close()

    # initialize scenario runner
    scenario_args = {'version': '0.9.12',
                     'host': '127.0.0.1',
                     'port': 2000,
                     'timeout': '60',
                     'sync': True,
                     'list': False,
                     'scenario': scenario_name,
                     'file': scenario_name + '_lav_',
                     'reloadWorld': False,
                     'configFile': scenario_name + '.xml',
                     'waitForEgo': False,
                     }
    if model == None:
        scenario_args['agent'] = None
        scenario_args['agentConfig'] = None
    elif model == 'auto':
        scenario_args['agent'] = './carla-expert/team_code/expert_agent/auto/auto_expert.py'
        scenario_args['agentConfig'] = './carla-expert/team_code/config/agent_config/mmfn.yaml'
    elif model == 'mmfn':
        scenario_args['agent'] = './carla-expert/team_code/expert_agent/mmfn/mmfn_expert.py'
        scenario_args['agentConfig'] = './carla-expert/team_code/config/agent_config/mmfn.yaml'
    elif model == 'lav':
        scenario_args['agent'] =  './LAV/team_code_v2/lav_agent.py'
        scenario_args['agentConfig'] = './LAV/team_code_v2/config.yaml'
    elif model == 'carla_ad' or model == 'autoware':
        scenario_args['waitForEgo'] = True
        scenario_args['agent'] = None
        scenario_args['agentConfig'] = None
        if model == 'autoware':
            scenario_args['timeout'] = '90'

    srunner_args = SrunnerArgs(scenario_args)
    srunner = ScenarioRunner(srunner_args)

    # generate scenario instance
    scenario_configuration = gen_scenario_config(scenario_name, actor_info['ego_vehicle']['route'])
    gps_route, route = interpolate_trajectory(world, scenario_configuration.trajectory)
    # draw_waypoints(world, route, vertical_shift=1.0, persistency=50000.0)

    scenario_configuration.route = route
    scenario_configuration.gps_route = gps_route

    srunner._load_and_wait_for_world(scenario_configuration.town, scenario_configuration.ego_vehicles)

    srunner._prepare_ego_vehicles(scenario_configuration.ego_vehicles)
    scenario = gen_scenario(scenario_configuration, scenario_name, world, srunner.ego_vehicles, scenario_config,
                            actor_info, route_info=route_info, timeout=scenario_args['timeout'])

    # run scenario
    # if model == 'autoware':
    #     time.sleep(5)
    result = srunner.run_parsed_scenario(scenario_configuration, scenario)
    print(result)
    # result_file.write('{}:{}:{}\n'.format(scenario_yaml, current_map, result))
    srunner.destroy()
    # return result


def generate_scenario_xml(map_name, actor_info, weather_info, file_name=None):
    root = ET.Element("scenarios")
    scenario = ET.SubElement(root, "scenario", name=file_name)
    scenario.set('type', file_name)
    scenario.set('town', map_name)
    # set ego vehicle spawn point
    ego_spawn_point = actor_info['ego_vehicle']['spawn_point']
    ego_vehicle = ET.SubElement(scenario, "ego_vehicle")
    ego_vehicle.set('model', 'vehicle.tesla.model3')
    ego_vehicle.set('x', str(ego_spawn_point.location.x))
    ego_vehicle.set('y', str(ego_spawn_point.location.y))
    ego_vehicle.set('z', str(ego_spawn_point.location.z))
    ego_vehicle.set('yaw', str(ego_spawn_point.rotation.yaw))

    # set weather
    weather = ET.SubElement(scenario, "weather")
    if weather_info:
        weather.set('cloudiness', weather_info['cloudiness'])
        weather.set('precipitation', weather_info['precipitation'])
        weather.set('precipitation_deposits', weather_info['precipitation_deposits'])
        weather.set('wind_intensity', weather_info['wind_intensity'])
        weather.set('sun_altitude_angle', weather_info['sun_altitude_angle'])
        weather.set('sun_azimuth_angle', weather_info['sun_azimuth_angle'])
    else:
        weather.set('cloudiness', "0")
        weather.set('precipitation', "0")
        weather.set('precipitation_deposits', "0")
        weather.set('wind_intensity', "0")
        weather.set('sun_altitude_angle', "25")
        weather.set('sun_azimuth_angle', "0")

    # set other actors' spawn points
    if len(actor_info['other_actors']) != 0:
        for actor in actor_info['other_actors']:
            actor_node = ET.SubElement(scenario, "other_actor")
            actor_node.set('x', str(actor['spawn_point'].location.x))
            actor_node.set('y', str(actor['spawn_point'].location.y))
            actor_node.set('z', str(actor['spawn_point'].location.z))
            actor_node.set('yaw', str(actor['spawn_point'].rotation.yaw))

            if 'type' not in actor.keys():
                actor_node.set('model', "vehicle.lincoln.mkz_2017")
            else:
                if actor['type'] == 'car':
                    actor_node.set('model', "vehicle.lincoln.mkz_2017")
                elif actor['type'] == 'pedestrian':
                    actor_node.set('model', "walker.pedestrian.0001")
                elif actor['type'] == 'school_bus':
                    actor_node.set('model', "vehicle.volkswagen.t2")

    tree = ET.ElementTree(root)
    tree.write(file_name + '.xml', xml_declaration=True)


def gen_npc_route(global_plan_gps, global_plan_world_coord):
    ds_ids = downsample_route(global_plan_world_coord, 1)
    route_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1])
                                     for x in ds_ids]
    route_plan = [global_plan_gps[x] for x in ds_ids]
    return route_plan, route_world_coord


class ParsedScenario(BasicScenario):
    def __init__(self, name, ego_vehicles, scenario_config, behavior_config, actor_info, world, route_info=None,
                 debug_mode=False, terminate_on_failure=False, criteria_enable=True, timeout=60):
        self.timeout = timeout
        print('timeout: ', timeout)
        self.world = world
        self.behavior_config = behavior_config
        self.actor_info = actor_info
        self.route_info = route_info

        super(ParsedScenario, self).__init__(name,
                                             ego_vehicles,
                                             scenario_config,
                                             world,
                                             debug_mode,
                                             criteria_enable=criteria_enable,
                                             terminate_on_failure=terminate_on_failure,
                                             timeout=self.timeout)

        # traffic_lights = CarlaDataProvider.get_traffic_lights()
        # for traffic_light in traffic_lights:
        #     traffic_light.set_state(carla.TrafficLightState.Green)
        #     traffic_light.set_green_time(60)
        #     traffic_light.set_red_time(1)
        #     traffic_light.set_yellow_time(1)
        # traffic_light.freeze(True)
        # if 'signal' in self.actor_info['ego_vehicle']:
        #     self.traffic_light = CarlaDataProvider.get_traffic_light_by_id(self.actor_info['ego_vehicle']['signal'][0])
        # if self.traffic_light:
        #     self.traffic_light.set_state(carla.TrafficLightState.Red)
        #     self.traffic_light.set_red_time(10)
        #     self.traffic_light.set_green_time(3)
        #     self.traffic_light.set_yellow_time(3)
        # self.traffic_light.freeze()
        # light_waypoints = self.traffic_light.get_affected_lane_waypoints()
        # draw_waypoints(world, [light_waypoint.transform for light_waypoint in light_waypoints], 1.0)

    def _create_behavior(self):
        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        if not self.behavior_config['Road network'] or (self.behavior_config['Road network'] and 'traffic light' != self.behavior_config['Road network']['Traffic signs']):
            traffic_light_behavior = py_trees.composites.Sequence(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            traffic_lights = CarlaDataProvider.get_traffic_lights()
            for traffic_light in traffic_lights:
                traffic_light.set_red_time(3)
            # traffic_light_behavior.add_child(TrafficLightControl(traffic_lights, carla.TrafficLightState.Red, True))
            # traffic_light_behavior.add_child(TrafficLightControl(traffic_lights, carla.TrafficLightState.Off, True))
            traffic_light_behavior.add_child(TrafficLightControl(traffic_lights, carla.TrafficLightState.Green, True))
            traffic_light_behavior.add_child(Idle(60))
            root.add_child(traffic_light_behavior)
        else:
            traffic_light_behavior = py_trees.composites.Sequence(
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            traffic_lights = CarlaDataProvider.get_traffic_lights()
            traffic_light_behavior.add_child(TrafficLightControl(traffic_lights, carla.TrafficLightState.Red, False))
            traffic_light_behavior.add_child(Idle(60))
            root.add_child(traffic_light_behavior)

        # behaviors for environment
        if self.behavior_config['Environment']:
            # for k, v in self.behavior_config['environment'].items():
            #     if k == 'night' and v['mode'] == 'transitional':
            #         weather_behavior = py_trees.composites.Sequence(
            #             policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            #         for i in range(100):
            #             weather_behavior.add_child(ChangeWeather(update_night(self.world, i)))
            #             weather_behavior.add_child(Idle(0.2))
            if 'nighttime' == self.behavior_config['Environment']['Time']:
                weather_behavior = py_trees.composites.Sequence(
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                for i in range(100):
                    weather_behavior.add_child(ChangeWeather(update_night(self.world, i)))
                    weather_behavior.add_child(Idle(0.2))
                root.add_child(weather_behavior)

            if 'foggy' == self.behavior_config['Environment']['Weather']:
                weather_behavior = py_trees.composites.Sequence(
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                for i in range(100):
                    weather_behavior.add_child(ChangeWeather(update_fog(self.world, i)))
                    weather_behavior.add_child(Idle(0.2))
                root.add_child(weather_behavior)

            if 'rainy' == self.behavior_config['Environment']['Weather']:
                weather_behavior = py_trees.composites.Sequence(
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                for i in range(100):
                    weather_behavior.add_child(ChangeWeather(update_rainy(self.world, i)))
                    weather_behavior.add_child(Idle(0.2))
                root.add_child(weather_behavior)

            if 'dusty' == self.behavior_config['Environment']['Weather']:
                weather_behavior = py_trees.composites.Sequence(
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                for i in range(100):
                    weather_behavior.add_child(ChangeWeather(update_dust(self.world, i)))
                    weather_behavior.add_child(Idle(0.2))
                root.add_child(weather_behavior)

            if 'wet' == self.behavior_config['Environment']['Weather']:
                weather_behavior = py_trees.composites.Sequence(
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                for i in range(100):
                    weather_behavior.add_child(ChangeWeather(update_wet(self.world, i)))
                    weather_behavior.add_child(Idle(0.2))
                root.add_child(weather_behavior)



        # behaviors for other actors
        if len(self.actor_info['other_actors']) > 0:
            for i, actor_info in enumerate(self.actor_info['other_actors']):
                if 'goal' in actor_info:

                    start_location = actor_info['spawn_point'].location
                    goal_location = actor_info['goal'].location
                    gps_route, route = interpolate_trajectory(self.world, [start_location, goal_location])
                    _, actor_plan_temp  = gen_npc_route(gps_route, route)
                    actor_plan = []
                    for p in actor_plan_temp:
                        waypoint = CarlaDataProvider.get_map().get_waypoint(p[0].location)
                        actor_plan.append((waypoint, RoadOption.LANEFOLLOW))
                    actor_behavior = py_trees.composites.Sequence(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                    root.add_child(actor_behavior)

                    if actor_info['type'] == 'pedestrian':
                        waypoint = CarlaDataProvider.get_map().get_waypoint(
                            actor_info['goal'].location)
                        goal = [(waypoint, RoadOption.LANEFOLLOW)]
                        waypoint_follower = WaypointFollower(self.other_actors[i], 1, plan=goal,
                                                             avoid_collision=False)
                        start_condition = InTriggerDistanceToLocation(
                            self.ego_vehicles[0],
                            self.actor_info['ego_vehicle']['route'][-1].start_waypoint.transform.location,
                            30,
                            name="Waiting for start position")
                    else:
                        if 'change lane to left' != self.behavior_config['Actors']['Ego vehicle']['Behavior'] and 'change lane to right' != \
                                self.behavior_config['Actors']['Ego vehicle']['Behavior']:
                            waypoint_follower = WaypointFollower(self.other_actors[i], 2.5, plan=actor_plan,
                                                                 avoid_collision=False)
                        else:
                            waypoint_follower = WaypointFollower(self.other_actors[i], 9, plan=actor_plan,
                                                                 avoid_collision=False)

                    start_condition = InTriggerDistanceToLocation(
                        self.ego_vehicles[0],
                        self.actor_info['ego_vehicle']['route'][-1].start_waypoint.transform.location,
                        20,
                        name="Waiting for start position")
                    driving_distance = DriveDistance(
                        self.other_actors[0],
                        50,
                        name="Distance")
                    if 'trigger' in actor_info and actor_info['trigger']:
                        actor_behavior.add_child(start_condition)
                    actor_behavior.add_child(waypoint_follower)
                    actor_behavior.add_child(driving_distance)
                else:
                    if 'status' in actor_info:
                        if actor_info['status'] == 'flash_light':
                            flash_status = carla.VehicleLightState.Special1
                            status_behavior = VehicleLightStateSetter(flash_status)
                            actor_behavior.add_child(status_behavior)

        end_condition = InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                    self.actor_info['ego_vehicle']['route'][
                                                        -1].end_waypoint.transform.location,
                                                    3,
                                                    name="achieve destination")

        print('end condition: ', self.actor_info['ego_vehicle']['route'][-1].end_waypoint.transform.location)

        root.add_child(end_condition)
        # py_trees.display.render_dot_tree(root)
        return root

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0], terminate_on_failure=self.terminate_on_failure)
        criteria.append(collision_criterion)

        keep_clear_criterion = KeepClearTest(self.ego_vehicles[0], terminate_on_failure=self.terminate_on_failure)
        criteria.append(keep_clear_criterion)

        if self.route_info and self.route_info['stop_sign']:
            stop_sign_criterion = RunningStopTest(self.ego_vehicles[0],
                                                  list_stop_signs=self.route_info['stop_sign'],
                                                  terminate_on_failure=self.terminate_on_failure)
            criteria.append(stop_sign_criterion)

        if self.route_info and self.route_info['traffic_light']:
            red_light_criterion = RunningRedLightTest(self.ego_vehicles[0],
                                                      terminate_on_failure=self.terminate_on_failure)
            criteria.append(red_light_criterion)

        if self.behavior_config['Oracle'] and 'yield' == self.behavior_config['Oracle']['longitudinal']:
            give_way_criterion = GiveWayTest(self.ego_vehicles[0], self.actor_info['ego_vehicle']['route'][
                    -1].start_waypoint,
                                             terminate_on_failure=self.terminate_on_failure)
            criteria.append(give_way_criterion)

        if self.behavior_config['Oracle'] and 'keep lane' == self.behavior_config['Oracle']['lateral']:
            keep_lane_criterion = KeepLaneTest(self.ego_vehicles[0], terminate_on_failure=self.terminate_on_failure)
            criteria.append(keep_lane_criterion)

        if self.behavior_config['Oracle'] and 'decelerate' == self.behavior_config['Oracle']['longitudinal']:
            decelerate_criterion = DecelerateTest(self.ego_vehicles[0], terminate_on_failure=self.terminate_on_failure)
            criteria.append(decelerate_criterion)

        if self.behavior_config['Oracle'] and 'no turn left' == self.behavior_config['Oracle']['lateral']:
            no_turn_left_criterion = NoTurnTest(self.ego_vehicles[0],
                                                    turn_location=self.actor_info['ego_vehicle']['route'][
                                                        -1].end_waypoint.transform.location,
                                                    terminate_on_failure=self.terminate_on_failure)
            criteria.append(no_turn_left_criterion)

        keep_distance_criterion = KeepDistanceTest(self.ego_vehicles[0], self.other_actors,
                                                   terminate_on_failure=self.terminate_on_failure)
        criteria.append(keep_distance_criterion)
        return criteria


def identify_actor_info(routes, config, map_info, world=None):
    actor_info = {'other_actors': []}
    filtered_routes = []
    conditions = []

    if not config['Road network']:
        for route in routes:
            if not route.has_crosswalk and not route.has_stop_sign and not route.has_traffic_light \
                    and not route.has_speed_sign:
                filtered_routes.append(route)

    else:
        # hardcode for two roundabout scenarios
        if 'roundabout' == config['Road network']['Road type']:
            ego_info = config['Actors']['Ego vehicle']
            if ego_info['Position']['Position relation'] == 'behind' and ('Other actor 1' not in config['Actors'] or config['Actors']['Other actor 1']['Type'] is None):
                spawn_point, goal = create_specific_route(routes, 30, 5, 28, -2)
                ego_route = Route(999, spawn_point, goal, map_info)
                # predecessor_route = ego_route.get_predecessor_route(routes)
                actor_info['ego_vehicle'] = {'spawn_point': ego_route.start_waypoint.transform,
                                             'route': [ego_route],
                                             'goal': goal.transform}
                return actor_info

            else:
                spawn_point_ego, goal_ego = create_specific_route(routes, 1772, 4, 8, 5)
                spawn_point_npc, goal_npc = create_specific_route(routes, 1786, -4, 8, 5)
                ego_route = Route(999, spawn_point_ego, goal_ego, map_info)
                predecessor_route = ego_route.get_predecessor_route(routes)
                actor_info['ego_vehicle'] = {'spawn_point': predecessor_route.start_waypoint.transform,
                                             'route': [predecessor_route, ego_route],
                                             'goal': goal_ego.transform}
                actor_info['other_actors'].append({'spawn_point': spawn_point_npc.previous(5)[0].transform,
                                                   'route': [
                                                       Route(-999, spawn_point_npc.previous(5)[0], goal_npc, map_info)],
                                                   'type': 'car',
                                                   'goal': goal_npc.transform,
                                                   'trigger': True})

                return actor_info

        conditions = []
        if config['Road network']['Road marker'] != 'None':
            conditions.append(config['Road network']['Road marker'])
        if config['Road network']['Traffic signs'] != 'None':
            conditions.append(config['Road network']['Traffic signs'])
        if config['Road network']['Road type'] != 'None':
            conditions.append(config['Road network']['Road type'])

        # filter routes with needed road elements
        for route in routes:
            meet = True
            for condition in conditions:
                if condition == 'stop sign' and not route.has_stop_sign:
                    meet = False
                    break
                elif condition == 'crosswalk' and not route.has_crosswalk:
                    meet = False
                    break
                elif condition == 'traffic light' and not route.has_traffic_light:
                    meet = False
                    break
                elif (condition == 'speed limit sign' and not route.has_speed_sign) or \
                        (condition == 'speed limit sign' and route.has_speed_sign and route.length < 30):
                    meet = False
                    break
                elif condition == 'solid line':
                    if not 'solid' in str(route.road_mark_left).lower() or not 'solid' in str(
                            route.road_mark_right).lower():
                        meet = False
                        break
                elif condition == 'intersection':
                    intersect, intersect_route = route.is_intersection(routes)
                    if not intersect:
                        meet = False
                        break
                elif condition == 't-intersection':
                    intersect, intersect_route = route.is_t_intersection(routes)

                    if not intersect:
                        meet = False
                        break
                    else:
                        # draw_route(world, route)
                        # draw_route(world, intersect_route)
                        pass
                elif condition == 'one way road':
                    if not route.one_way:
                        meet = False
                        break
                elif condition == 'one lane road':
                    if not route.num_lanes == 1:
                        meet = False
                        break
                elif condition == 'two lane road':
                    if not route.num_lanes == 2:
                        meet = False
                        break
                elif condition == 'three lane road':
                    if not route.num_lanes == 3:
                        meet = False
                        break
                elif 'no turn left sign' in condition:
                    predecessor_route = route.get_predecessor_route(routes)
                    if not predecessor_route or predecessor_route.is_no_turn(map_info, routes) != condition:
                        meet = False
                        break
                    # else:
                        # intersect, intersect_route = predecessor_route.is_intersection(routes)
                        # if intersect:
                        #     draw_route(world, intersect_route)
                        # intersect_route = route.get_cross_routes(routes, position=-1)[0]
                        # draw_route(world, intersect_route)
                elif condition == 'do not enter sign':
                    if not route.do_not_enter:
                        meet = False
                        break

            if meet:
                filtered_routes.append(route)

                # draw_route(world, route)
                # successor_route = route.get_successor_route(routes)
                # if successor_route:
                #     draw_route(world, successor_route)
                #     print(successor_route.is_no_turn(map_info, routes))
                # draw_route(world, intersect_route)

        filtered_routes_new = []
        for route in filtered_routes:
            for another_route in routes:
                if abs(another_route.start_waypoint.transform.location.x - route.start_waypoint.transform.location.x) <= 1 and \
                        abs(another_route.start_waypoint.transform.location.y - route.start_waypoint.transform.location.y) <= 1 and \
                        another_route not in filtered_routes_new:
                    filtered_routes_new.append(another_route)

        filtered_routes = filtered_routes_new

    # identify route that meets requirements for actor behaviors
    ego_route = None
    ego_info = config['Actors']['Ego vehicle']
    if 'Other actor 1' in config['Actors']:
        other_actor_info = config['Actors']['Other actor 1']
    else:
        other_actor_info = None

    if not ego_info and not other_actor_info and len(filtered_routes) == 0:
        return None

    filtered_routes_2 = []
    if not ego_info:
        filtered_routes_2 = filtered_routes
    else:
        for route in filtered_routes:
            if config['Road network'] and 'no turn left sign' not in conditions and 'do not enter sign' not in conditions:
                if ego_info and DIRECTION[ego_info['Behavior']] == route.direction:

                    if ego_info['Behavior'] == 'change lane to right' and route.start_lane > 0 and \
                            route.start_lane >= route.num_lanes:
                        filtered_routes_2.append(route)
                    else:
                        filtered_routes_2.append(route)
            else:
                filtered_routes_2.append(route)

    for route in filtered_routes_2:


        ego_route = route
        driving_routes = [ego_route]
        if ego_info['Behavior'] == 'None':
            ego_info['Behavior'] = 'go forward'
        if ego_info and 'change lane to left' != ego_info['Behavior'] and 'change lane to right' != ego_info['Behavior']:
            route_length = 0
            while True:
                current_route = driving_routes[0].get_predecessor_route(routes)
                if current_route:
                    driving_routes.insert(0, current_route)
                    route_length += current_route.length
                else:
                    break
                if route_length >= 30 and current_route.direction == 0:
                    break
        else:
            if (ego_info['Behavior'] == 'change lane to right' and route.start_lane > 0 and \
                route.start_lane >= route.num_lanes):
                driving_routes.append(driving_routes[0].get_successor_route(routes))
            else:
                continue

        waypoint = None
        # identify the position of the other actor
        if not other_actor_info or other_actor_info['Type'] is None:
            break

        elif other_actor_info['Position']['Position reference'] is None:
            other_actor_info['Position']['Position reference'] = 'ego vehicle'
            other_actor_info['Position']['Position relation'] = 'front'
            other_actor_info['Behavior'] = 'go forward'

        if other_actor_info['Position']['Position reference'] == 'ego vehicle':
            if other_actor_info['Position']['Position relation'] == 'front':
                if 'crosswalk' in conditions or 'intersection' in conditions or 't-intersection' in conditions:
                    if other_actor_info['Type'] == 'car':
                        waypoint = driving_routes[-1].start_waypoint.next(5)[0]
                    else:
                        waypoint = driving_routes[-1].start_waypoint.next(10)[0]
                else:
                    if other_actor_info['Type'] == 'car':
                        waypoint = driving_routes[0].start_waypoint.next(10)[0]
                    else:
                        waypoint = driving_routes[0].start_waypoint.next(15)[0]

                other_actor_route = {'spawn_point': waypoint.transform,
                                     'route': [route],
                                     'type': other_actor_info['Type'],
                                     'trigger': False}
                other_actor_route['spawn_point'].location.z = 0.5
                actor_info['other_actors'].append(other_actor_route)

            elif other_actor_info['Position']['Position relation'] == 'behind':
                other_actor_route = {'spawn_point': driving_routes[0].start_waypoint.previous(10)[0].transform,
                                     'route': [route],
                                     'type': other_actor_info['Type'],
                                     'trigger': False}
                waypoint = driving_routes[0].start_waypoint.previous(10)[0]
                other_actor_route['spawn_point'].location.z = 0.5
                actor_info['other_actors'].append(other_actor_route)

            else:
                if 'left' == other_actor_info['Position']['Position relation']:
                    waypoint = driving_routes[0].start_waypoint.get_left_lane()
                elif 'right' == other_actor_info['Position']['Position relation']:
                    waypoint = driving_routes[0].start_waypoint.get_right_lane()
                elif 'left_behind' == other_actor_info['Position']['Position relation']:
                    waypoint = driving_routes[0].start_waypoint.previous(10)[0].get_left_lane()
                elif 'right_behind' == other_actor_info['Position']['Position relation']:
                    waypoint = driving_routes[0].start_waypoint.previous(10)[0].get_right_lane()

                if not waypoint or waypoint.lane_type != carla.LaneType.Driving:
                    continue
                else:
                    other_actor_route = {'spawn_point': waypoint.transform,
                                         'route': [route],
                                         'type': other_actor_info['Type'],
                                         'trigger': False}

                    other_actor_route['spawn_point'].location.z = 0.5
                    actor_info['other_actors'].append(other_actor_route)

        elif 'intersection' in other_actor_info['Position']['Position reference']:

            if other_actor_info['Position']['Position relation'] == 'behind':
                waypoint = ego_route.start_waypoint

            elif other_actor_info['Position']['Position relation'] == 'front':
                waypoint = ego_route.end_waypoint.next(5)[0]

            else:
                if other_actor_info['Type'] != 'pedestrian':
                    if 'in' == other_actor_info['Position']['Position relation'] or 'on' == other_actor_info['Position']['Position relation']:
                        cross_route = ego_route.get_cross_routes(routes,
                                                                 direction=DIRECTION[other_actor_info['Behavior']],
                                                                 position=DIRECTION[
                                                                     'right'])
                    else:
                        cross_route = ego_route.get_cross_routes(routes,
                                                                 direction=DIRECTION[other_actor_info['Behavior']],
                                                                 position=DIRECTION[other_actor_info['Position']['Position relation']])
                    if cross_route:
                        if 'in' == other_actor_info['Position']['Position relation'] or 'on' == other_actor_info['Position']['Position relation']:
                            other_actor_route = {'spawn_point': cross_route[0].start_waypoint.next(8)[0].transform,
                                                 'goal': cross_route[0].end_waypoint.transform,
                                                 'route': cross_route,
                                                 'type': other_actor_info['Type'],
                                                 'trigger': True}
                        else:
                            other_actor_route = {'spawn_point': cross_route[0].start_waypoint.transform,
                                                 'goal': cross_route[0].end_waypoint.transform,
                                                 'route': cross_route,
                                                 'type': other_actor_info['Type'],
                                                 'trigger': True}
                        other_actor_route['spawn_point'].location.z = 0.5
                        actor_info['other_actors'].append(other_actor_route)
                        print('other_actor_spawn_point', other_actor_info['Position']['Position relation'], other_actor_route['spawn_point'])
                        break
                else:
                    if 'static' == other_actor_info['Behavior']:
                        direction = 'static'
                    elif 'in' == other_actor_info['Position']['Position relation'] or 'on' == other_actor_info['Position']['Position relation']:
                            direction = 'right'
                    else:
                        direction = other_actor_info['position_relation']
                    waypoint_start, waypoint_end = get_roadside(ego_route, map_info, direction, world)
                    if 'in' == other_actor_info['Position']['Position relation'] or 'on' == other_actor_info['Position']['Position relation']:
                        other_actor_route = {'spawn_point': waypoint_start.next(3)[0].transform,
                                             'goal': waypoint_end.transform,
                                             'type': other_actor_info['Type'],
                                             'trigger': True}
                    else:
                        other_actor_route = {'spawn_point': waypoint_start.transform,
                                             'goal': waypoint_end.transform,
                                             'type': other_actor_info['Type'],
                                             'trigger': True}
                    actor_info['other_actors'].append(other_actor_route)
                    break
            if not waypoint:
                return None
            if not actor_info['other_actors']:

                if other_actor_info['Behavior'] != 'static':

                    other_actor_route = {'spawn_point': waypoint.transform,
                                            'goal': ego_route.end_waypoint.transform,
                                            'route': ego_route,
                                            'type': other_actor_info['Type'],
                                            'trigger': True}
                    other_actor_route['spawn_point'].location.z = 0.5
                    actor_info['other_actors'].append(other_actor_route)
                    break
                    # else:
                    #     if other_actor_info['behavior'][1] == 'flash_light':
                    #         other_actor_route = {'spawn_point': waypoint.transform,
                    #                              'goal': ego_route.end_waypoint.transform,
                    #                              'route': ego_route,
                    #                              'type': other_actor_info['type'],
                    #                              'status': 'flash_light',
                    #                              'trigger': True}
                    #         other_actor_route['spawn_point'].location.z = 0.5
                    #         actor_info['other_actors'].append(other_actor_route)
                    #         break
                else:
                    other_actor_route = {'spawn_point': waypoint.transform,
                                         'route': ego_route,
                                         'type': other_actor_info['Type'],
                                         'trigger': True}
                    other_actor_route['spawn_point'].location.z = 0.5
                    actor_info['other_actors'].append(other_actor_route)


        elif 'crosswalk' == other_actor_info['Position']['Position reference']:
            if 'behind' == other_actor_info['Position']['Position relation']:
                waypoint = ego_route.start_waypoint.previous(3)[0]
                # waypoint = ego_route.start_waypoint

            elif 'front' == other_actor_info['Position']['Position relation']:
                waypoint = ego_route.start_waypoint.next(3)[0]

            elif other_actor_info['Type'] != 'pedestrian':
                if 'static' == other_actor_info['Behavior']:
                    direction = 'static'
                elif 'in' == other_actor_info['Position']['Position relation'] or 'on' == other_actor_info['Position']['Position relation']:
                    direction = 'right'
                else:
                    direction = other_actor_info['Position']['Position relation']

                waypoint_start, waypoint_end = get_roadside(ego_route, map_info, direction, world)
                if 'in' == other_actor_info['Position']['Position relation'] or 'on' == other_actor_info['Position']['Position relation']:
                    waypoint_start = waypoint_start.next(3)[0]
                other_actor_route = {'spawn_point': waypoint_start.transform,
                                     'goal': waypoint_end.transform,
                                     'type': other_actor_info['Type'],
                                     'trigger': True}
                actor_info['other_actors'].append(other_actor_route)
                break
            elif other_actor_info['Type'] == 'pedestrian':
                if 'in' == other_actor_info['Position']['Position relation'] or 'on' == other_actor_info['Position']['Position relation']:
                    direction = 'right'
                else:
                    direction = other_actor_info['Position']['Position relation']
                waypoint_start, waypoint_end = get_roadside(ego_route, map_info, direction, world)
                if 'in' == other_actor_info['Position']['Position relation'] or 'on' == other_actor_info['Position']['Position relation']:
                    waypoint_start = waypoint_start.next(3)[0]

                other_actor_route = {'spawn_point': waypoint_start.transform,
                                     'goal': waypoint_end.transform,
                                     'type': other_actor_info['Type'],
                                     'trigger': True}
                actor_info['other_actors'].append(other_actor_route)
                break

            if not actor_info['other_actors']:
                if other_actor_info['behavior'][0] != 'static':
                    other_actor_route = {'spawn_point': waypoint.transform,
                                         'goal': ego_route.end_waypoint.transform,
                                         'route': ego_route,
                                         'type': other_actor_info['Type'],
                                         'trigger': True}
                    other_actor_route['spawn_point'].location.z = 0.5
                    actor_info['other_actors'].append(other_actor_route)
                    break
                else:
                    other_actor_route = {'spawn_point': waypoint.transform,
                                         'route': ego_route,
                                         'type': other_actor_info['Type'],
                                         'trigger': True}
                    other_actor_route['spawn_point'].location.z = 0.5
                    actor_info['other_actors'].append(other_actor_route)

        # identify the behavior of the other actor
        if other_actor_info['Behavior'] != 'static':
            if 'front' in other_actor_info['Position']['Position relation']:
                # goal = waypoint.next(50)[0]
                goal = other_actor_route['route'][0].end_waypoint.next(20)[0]
            elif 'left' not in other_actor_info['Position']['Position relation'] and 'right' not in other_actor_info[
                'Position']['Position relation']:
                goal = ego_route.end_waypoint
            elif 'left' in other_actor_info['Position']['Position relation']:
                goal = ego_route.end_waypoint.get_left_lane()
            elif 'right' in other_actor_info['Position']['Position relation']:
                goal = ego_route.end_waypoint.get_right_lane()

            actor_route = Route('actor', waypoint, goal, map_info)
            actor_info['other_actors'][0]['goal'] = goal.transform
            actor_info['other_actors'][0]['route'] = actor_route
            break
        elif len(actor_info['other_actors']) != 0:
            break

    if ego_route:
        if ego_info and ego_info['Behavior'] == 'change lane to left':
            successor_route = ego_route.get_successor_route(routes)
            ego_route_new = Route(999, ego_route.start_waypoint, successor_route.end_waypoint.get_left_lane(), map_info)
        elif ego_info and ego_info['Behavior'] == 'change lane to right':
            successor_route = ego_route.get_successor_route(routes)
            ego_route_new = Route(999, ego_route.start_waypoint, successor_route.end_waypoint.get_right_lane(),
                                  map_info)
        elif ego_info and ego_info['Behavior'] == 'turn left' and 'no turn left sign' == config['Road network']['Traffic signs']:
            intersect_route = ego_route.get_cross_routes(routes, position=-1)[0]
            ego_route_new = Route(999, ego_route.start_waypoint, intersect_route.end_waypoint, map_info)
        elif ego_info and ego_info['Behavior'] == 'turn left' and 'do not enter sign' == config['Road network']['Traffic signs']:
            intersect_route = ego_route.get_cross_routes(routes, direction=1, position=1)[0]
            ego_route_new = Route(999, ego_route.start_waypoint, intersect_route.start_waypoint, map_info)
        else:
            ego_route_new = ego_route

        driving_routes = [ego_route_new]
        if ego_info and 'change lane to left' != ego_info['Behavior'] and 'change lane to right' != ego_info['Behavior']:
            route_length = 0
            while True:
                current_route = driving_routes[0].get_predecessor_route(routes)
                if current_route:
                    driving_routes.insert(0, current_route)
                    route_length += current_route.length
                    # print('start point update:', current_route.start_waypoint.transform.location)
                    # break
                    # print('driving_routes', driving_routes)

                else:
                    break
                if route_length >= 30 and current_route.direction == 0:
                    break

        elif not ego_info:
            route_length = 0
            while True:
                current_route = driving_routes[0].get_predecessor_route(routes)
                if current_route:
                    driving_routes.insert(0, current_route)
                    route_length += current_route.length
                else:
                    break
                if route_length >= 30 and current_route.direction == 0:
                    break


        # predecessor_route = ego_route.get_predecessor_route(routes)
        actor_info['ego_vehicle'] = {'spawn_point': driving_routes[0].start_waypoint.transform,
                                     'route': driving_routes,
                                     'goal': driving_routes[-1].end_waypoint.transform}
        actor_info['ego_vehicle']['spawn_point'].location.z = 0.5
        if other_actor_info and 'Position' in other_actor_info and 'ego vehicle' == other_actor_info['Position']['Position reference'] and other_actor_info['Position']['Position relation'] != 'left' and other_actor_info['Position']['Position relation'] != 'right':
            actor_info['other_actors'][0]['spawn_point'] = driving_routes[0].start_waypoint.next(10)[0].transform
            # print('other_actor_spawn_point', actor_info['other_actors'][0]['spawn_point'])
            # actor_info['other_actors'][0]['spawn_point'] = driving_routes[0].start_waypoint.transform
            actor_info['other_actors'][0]['spawn_point'].location.z = 0.5

        if 'traffic light' in conditions:
            actor_info['ego_vehicle']['signal'] = ego_route.signals
    else:
        return None

    return actor_info


def update_rainy(world, count):
    new_weather = world.get_weather()
    new_weather.precipitation = min(count * 3, 100)
    return Weather(new_weather)


def update_wet(world, count):
    new_weather = world.get_weather()
    # new_weather.precipitation = min(count * 10, 100)
    new_weather.wetness = min(count * 10, 100)
    return Weather(new_weather)


def update_fog(world, count):
    new_weather = world.get_weather()
    new_weather.fog_density = min(count * 3, 100)
    return Weather(new_weather)


def update_dust(world, count):
    new_weather = world.get_weather()
    new_weather.dust_storm = min(count * 3, 100)
    return Weather(new_weather)


def update_night(world, count):
    new_weather = world.get_weather()
    # new_weather.sun_azimuth_angle -= count * 3
    new_weather.sun_altitude_angle -= count * 3 / 4
    # print(new_weather.sun_azimuth_angle, new_weather.sun_altitude_angle)
    return Weather(new_weather)


def draw_route(world, route):
    # print(route.direction)
    # next_waypoint = route.start_waypoint.next(distance=5)
    # next_waypoint = route.start_waypoint.next(distance=5)
    lane_width = route.start_waypoint.lane_width

    world.debug.draw_string(route.start_waypoint.transform.location, 'o-start', draw_shadow=False,
                            color=carla.Color(r=0, g=255, b=0), life_time=120,
                            persistent_lines=True)
    world.debug.draw_string(route.end_waypoint.transform.location, 'o-end', draw_shadow=False,
                            color=carla.Color(r=0, g=255, b=0), life_time=120,
                            persistent_lines=True)
    world.debug.draw_line(route.start_waypoint.transform.location, route.end_waypoint.transform.location,
                          life_time=120)
    # world.debug.draw_point(location=next_waypoint[0].transform.location, size=0.5, color=(255, 0, 0), life_time=120, persistent_lines=False)


def draw_waypoints(world, waypoints, vertical_shift, persistency=-1):
    """
    Draw a list of waypoints at a certain height given in vertical_shift.
    """
    for w in waypoints:
        wp = w[0].location + carla.Location(z=vertical_shift)

        size = 0.2
        if w[1] == RoadOption.LEFT:  # Yellow
            color = carla.Color(255, 255, 0)
        elif w[1] == RoadOption.RIGHT:  # Cyan
            color = carla.Color(0, 255, 255)
        elif w[1] == RoadOption.CHANGELANELEFT:  # Orange
            color = carla.Color(255, 64, 0)
        elif w[1] == RoadOption.CHANGELANERIGHT:  # Dark Cyan
            color = carla.Color(0, 64, 255)
        elif w[1] == RoadOption.STRAIGHT:  # Gray
            color = carla.Color(128, 128, 128)
        else:  # LANEFOLLOW
            color = carla.Color(0, 255, 0)  # Green
            size = 0.1

        world.debug.draw_point(wp, size=size, color=color, life_time=persistency)

    world.debug.draw_point(waypoints[0][0].location + carla.Location(z=vertical_shift), size=0.2,
                           color=carla.Color(0, 0, 255), life_time=persistency)
    world.debug.draw_point(waypoints[-1][0].location + carla.Location(z=vertical_shift), size=0.2,
                           color=carla.Color(255, 0, 0), life_time=persistency)


# run('scenario_config/signs/rule6.yaml')
# run('scenario_config/right_of_way/rule10.yaml')

def exp():
    exp_results = {}
    rules = {'right_of_way': ['rule1', 'rule2', 'rule3', 'rule7', 'rule8', 'rule9', 'rule10'],
             'signs': ['rule1', 'rule2', 'rule3', 'rule4', 'rule5', 'rule6'],
             'speed': ['rule1', 'rule2', 'rule3', 'rule4', 'rule5']}

    models = ['mmfn']
    for model in models:
        for k, v in rules.items():
            exp_results[k] = {}
            for rule in v:
                exp_results[k][rule] = []
                for i in range(5):
                    result = run('scenario_config/{}/{}.yaml'.format(k, rule), model)
                    exp_results[k][rule].append(result)
                    time.sleep(10)

        with open('result/result_{}.pickle'.format(model), 'wb') as handle:
            pickle.dump(exp_results, handle, protocol=pickle.HIGHEST_PROTOCOL)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', default='scenarios/rule_1.yaml')
    parser.add_argument('-m', '--model', default='auto')
    parser.add_argument('-r', '--reload', default=1)
    parser.add_argument('-t', '--town', default=10)
    # parser.add_argument('-re', '--recording', default=1)

    args = parser.parse_args()
    print(args.model)

    run(args.file, args.model, args.reload, 'single', args.town)

