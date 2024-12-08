import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, InTriggerDistanceToLocation
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
import py_trees
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ChangeWeather, Idle, KeepVelocity, \
    WaypointFollower, ActorDestroy
from srunner.scenariomanager.weather_sim import Weather
from agents.navigation.local_planner import RoadOption
from srunner.scenariomanager.scenarioatomics.atomic_criteria import RunningStopTest, GiveWayTest


class Night(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, timeout=35 * 60):
        """
        Setup all relevant parameters and create scenario
        """
        self.config = config
        self.debug = debug_mode

        self.timeout = timeout  # Timeout of scenario in seconds
        self.world = world

        super(Night, self).__init__("NightScenario",
                                    ego_vehicles,
                                    config,
                                    world,
                                    debug_mode,
                                    terminate_on_failure=True,
                                    criteria_enable=True)

    def update_weather(self, count):
        new_weather = self.world.get_weather()
        # new_weather.sun_azimuth_angle -= count * 3
        new_weather.sun_altitude_angle -= count * 3 / 4
        print(new_weather.sun_azimuth_angle, new_weather.sun_altitude_angle)
        return Weather(new_weather)

    def _create_behavior(self):
        """
        Basic behavior do nothing, i.e. Idle
        """
        # weather = self.world.get_weather()
        # weather.sun_azimuth_angle = 180
        # weather.sun_altitude_angle = 90
        weathers = []
        for i in range(120):
            weathers.append(self.update_weather(i))

        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()

        change_weathers = [ChangeWeather(weather) for weather in weathers]

        weather_sequence =  py_trees.composites.Sequence(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # actor_crossing_parallel = py_trees.composites.Parallel(
        #     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        root.add_child(scenario_sequence)
        # root.add_child(weather_sequence)

        for change_weather in change_weathers:
            weather_sequence.add_child(change_weather)
            weather_sequence.add_child(Idle(0.25))

        if len(self.other_actors) != 0:
            waypoint = CarlaDataProvider.get_map().get_waypoint(carla.Location(x=27.142294, y=66.283257, z=0.000000))
            goal = [(waypoint, RoadOption.LANEFOLLOW)]

            continue_driving = py_trees.composites.Sequence(
                "ContinueDriving",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

            waypoint_follower = WaypointFollower(self.other_actors[0], 10, plan=goal,
                                                 avoid_collision=False)

            continue_driving_distance = DriveDistance(
                self.other_actors[0],
                50,
                name="Distance")

            continue_driving_timeout = TimeOut(5)

            startcondition = InTriggerDistanceToLocation(
                self.ego_vehicles[0],
                carla.Location(x=40.389641, y=52.594585, z=2.000000),
                5,
                name="Waiting for start position")

            continue_driving.add_child(startcondition)
            continue_driving.add_child(waypoint_follower)
            continue_driving.add_child(continue_driving_distance)
            continue_driving.add_child(continue_driving_timeout)

            wait = DriveDistance(
                self.ego_vehicles[0],
                50,
                name="DriveDistance")

            # actor_crossing_parallel.add_child(waypoint_follower)
            # keep_velocity = KeepVelocity(self.other_actors[0], 0)
            # scenario_sequence.add_child(actor_crossing_parallel)
            # scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))
            # keep_velocity_other_parallel.add_child(waypoint_follower)

            # scenario_sequence.add_child(startcondition)
            # scenario_sequence.add_child(sync_arrival_parallel)
            scenario_sequence.add_child(continue_driving)
            scenario_sequence.add_child(wait)
            scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        # criteria = []
        # stop_sign_criterion = RunningStopTest(self.ego_vehicles[0], terminate_on_failure=True)
        # give_way_criterion = GiveWayTest(self.ego_vehicles[0], carla.Location(x=40.389641, y=52.594585, z=2.000000) ,terminate_on_failure=True)
        # criteria.append(give_way_criterion)
        # return criteria
        pass

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
