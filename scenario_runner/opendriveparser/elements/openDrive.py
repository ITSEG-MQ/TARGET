
class OpenDrive(object):

    def __init__(self):
        self._header = Header()
        self._roads = []
        self._controllers = []
        self._junctions = []
        self._junctionGroups = []
        self._stations = []
        self._signals = {}
        self._has_speed_sign = False
        self._has_crosswalk = False

    @property
    def header(self):
        return self._header

    @property
    def roads(self):
        return self._roads

    def getRoad(self, id):
        for road in self._roads:
            if road.id == id:
                return road

        return None

    @property
    def controllers(self):
        return self._controllers

    @property
    def junctions(self):
        return self._junctions

    def getJunction(self, junctionId):
        for junction in self._junctions:
            if junction.id == junctionId:
                return junction
        return None

    @property
    def junctionGroups(self):
        return self._junctionGroups

    @property
    def stations(self):
        return self._stations

    @property
    def has_speed_sign(self):
        return self._has_speed_sign

    @has_speed_sign.setter
    def has_speed_sign(self, value):
        self._has_speed_sign = value

    @property
    def has_crosswalk(self):
        return self._has_crosswalk

    @has_crosswalk.setter
    def has_crosswalk(self, value):
        self._has_speed_sign = value

    @property
    def signal(self):
        return self._signals

    def get_road(self, road_id):
        target_road = None
        for road in self._roads:
            if road.id == road_id:
                target_road = road
                break

        return target_road


class Header(object):

    def __init__(self):
        self._revMajor = None
        self._revMinor = None
        self._name = None
        self._version = None
        self._date = None
        self._north = None
        self._south = None
        self._east = None
        self._west = None
        self._vendor = None

    @property
    def revMajor(self):
        return self._revMajor

    @revMajor.setter
    def revMajor(self, value):
        self._revMajor = value

    @property
    def revMinor(self):
        return self._revMinor

    @revMinor.setter
    def revMinor(self, value):
        self._revMinor = value

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = value

    @property
    def version(self):
        return self._version

    @version.setter
    def version(self, value):
        self._version = value

    @property
    def date(self):
        return self._date

    @date.setter
    def date(self, value):
        self._date = value

    @property
    def north(self):
        return self._north

    @north.setter
    def north(self, value):
        self._north = value

    @property
    def south(self):
        return self._south

    @south.setter
    def south(self, value):
        self._south = value

    @property
    def east(self):
        return self._east

    @east.setter
    def east(self, value):
        self._east = value

    @property
    def west(self):
        return self._west

    @west.setter
    def west(self, value):
        self._west = value

    @property
    def vendor(self):
        return self._vendor

    @vendor.setter
    def vendor(self, value):
        self._vendor = value
