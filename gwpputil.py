import math
import pathfind
import maputil

"""
    Represents static missions parameters which include
    the mission bounds, waypoints, search area bounds
    and static obstacles.
"""
class Mission_Param():
    def __init__(self, bnds, wps, srch_area, stat_obsts):
        self.bounds = bnds
        self.waypoints = wps
        self.search_area = srch_area
        self.static_obstacles = stat_obsts
        self.flightplan = None
        self.waypoints = self.waypoints# + self.generate_search_pattern()
        print(self.waypoints)

    """
        Returns north west and south east points of bounding rectangle.
    """
    def bounding_rect(self):
        east = self.bounds.vertices[0].x
        west = self.bounds.vertices[0].x
        north = self.bounds.vertices[0].y
        south = self.bounds.vertices[0].y
        for point in self.bounds.vertices:
            #Assuming North-West semihemisphere
            if point.x > west:
                west = point.x
            if point.x < east:
                east = point.x
            if point.y > north:
                north = point.y
            if point.y < south:
                south = point.y
        return [[west, north], [east, south]]

    """
        Generates a search pattern by filling the
        search area with lines at 2*image range apart.
        This function returns a series of waypoints which
        have been shifted from the search area bounds so as
        to not be placed on obstacles.
    """
    def generate_search_pattern(self):
        search_pattern = []
        image_range = 30
        #Find the highest and lowest y
        highest = self.search_area.vertices[0].y
        lowest = self.search_area.vertices[0].y
        for vert in self.search_area.vertices:
            if vert.y > highest:
                highest = vert.y
            if vert.y < lowest:
                lowest = vert.y
        #Convert list of vertices to [m, b, low y, high y]
        #where m is the slope of the line and b is the y intercept
        edges = []
        for i in range(1, len(self.search_area.vertices) + 1):
            p = self.search_area.vertices[i - 1]
            q = self.search_area.vertices[i % len(self.search_area.vertices)]
            m = (q.y - p.y)/(q.x - p.x)
            b = p.y - (m * p.x)
            if p.y < q.y:
                low = p.y
                high = q.y
            else:
                low = q.y
                high = p.y
            edges = edges + [[m, b, low, high]]
        #make a list of line segments crossing the search search area
        points = []
        lowest = lowest + image_range
        highest = highest - image_range
        reverse = False
        for y in range(int(lowest), int(highest), image_range*2):
            line = []
            for edge in edges:
                if y >= edge[2] and y <= edge[3]:
                    x = (y - edge[1])/edge[0]
                    line = line + [maputil.Point(x, y)]
            #Sort by x value
            for i in range(len(line) - 1):
                for j in range(1, len(line)):
                    if line[i].x > line[j].x:
                        temp = line[i]
                        line[i] = line[j]
                        line[j] = temp
            #Find all points within an obstacle, then move said point
            for i in range(len(line)):
                point = line[i]
                obst = None
                for o in self.static_obstacles:
                    if point.z <= o.point.z and pathfind.dist2d(point, o.point) <= o.radius + (pathfind.STEP * 1.5):
                        obst = o
                        break
                if obst:
                    #Find the x of the obstacle's edge at the y of the point
                    b = math.sqrt(((obst.radius + (pathfind.STEP * 4)) ** 2) - ((point.y - obst.point.y) ** 2))
                    if i % 2 == 0:
                        b = obst.point.x + b
                    else:
                        b = obst.point.x - b
                    point.x = b
            #Reverse direction of every other line
            if reverse:
                line = line[::-1]
            reverse = not reverse
            points = points + line

        return points

    """
        Generates a flightplan by running a pathfinding function
        between each sequential pair of waypoints. The flightplan
        variable will contain a list of all waypoints returned by
        calling said function on the points in their original order.
    """
    def generate_flightplan(self, start_pos, heading):
        self.flightplan = []
        points = self.waypoints
        prev = start_pos
        for i in range(len(points)):
            print(i)
            wp = points[i]
            self.flightplan = self.flightplan + pathfind.pathfind(prev, wp, self)
            prev = wp
        print("Generating path")

"""
    A unit converter utility
"""
class Deg_Met_Converter():
    def __init__(self, orig = None):
        self.origin = orig

    """
        Set origin for the cartesian coordinate system that
        will be used to convert latitude and longitude to metric units.
        All metric coordinates will be expressed as distances in meters from
        this point.
    """
    def set_orig(self, orig):
        self.origin = orig

    """
        Converts degrees latitude to meters. This conversion finds the east/west
        distance in meters between the supplied point and origin
    """
    def degreesLatToMeters(self, degrees):
        latR = math.radians(self.origin[0])
        return (degrees - self.origin[0]) * (111132.954 - (559.822 * math.cos(2 * latR)) +	(1.175 * math.cos(4 * latR)) - (0.0023 * math.cos(6 * latR)))

    """
        Converts degrees longitude to meters. This conversion finds the north/south 
        distance in meters between the supplied point and origin
    """
    def degreesLongToMeters(self, degrees):
        latR = math.radians(self.origin[0])
        return (degrees - self.origin[1]) * (111132.954 * math.cos(latR))

    def metersToDegreesLat(self, meters):
	    latR = math.radians(self.origin[0])
	    return (meters / (111132.954 - (559.822 * math.cos(2 * latR)) + (1.175 * math.cos(4 * latR)) - (0.0023 * math.cos(6 * latR)))) + self.origin[0]

    def metersToDegreesLong(self, meters):
        latR = math.radians(self.origin[0])
        return (meters / (111132.954 * math.cos(latR))) + self.origin[1]

    def feetToMeters(self, feet):
        return feet * 0.3048

    def metersToFeet(self, meters):
        return meters / 0.3048

def read_lat_long(file, conv):
    f = open(file, "r")
    data = f.read()
    f.close()
    file_chunks = data.split()
    data_ll = []
    for i in range(0, len(file_chunks), 2):
        data_ll.append([float(file_chunks[i]),float(file_chunks[i + 1])])
    if conv.origin == None:
        conv.origin = data_ll[0]
    data_met = []
    for i in range(len(data_ll)):
        data_met.append([conv.degreesLongToMeters(data_ll[i][1]), conv.degreesLatToMeters(data_ll[i][0])])
    for i in range(len(data_met)):
        data_met[i] = maputil.Point(data_met[i][0], data_met[i][1])
    return data_met

def read_obst_file(file, conv):
    f = open(file, "r")
    data = f.read()
    f.close()
    file_chunks = data.split()
    data_ll = []
    rads = []
    alts = []
    for i in range(0, len(file_chunks), 4):
        data_ll.append([float(file_chunks[i]),float(file_chunks[i + 1])])
        alts.append(float(conv.feetToMeters(float(file_chunks[i + 2]))))
        rads.append(float(conv.feetToMeters(float(file_chunks[i + 3]))))
    if conv.origin == None:
        conv.origin = data_ll[0]
    data_met = []
    for i in range(len(data_ll)):
        data_met.append([conv.degreesLongToMeters(data_ll[i][1]), conv.degreesLatToMeters(data_ll[i][0]), alts[i], rads[i]])
    for i in range(len(data_met)):
        data_met[i] = maputil.Obstacle(maputil.Point(data_met[i][0], data_met[i][1], data_met[i][2]), data_met[i][3])
    return data_met

def read_fence(fen_file, conv):
    f = open(fen_file, "r")
    fen_data = f.read()
    fen_data_ll = read_lat_long(fen_data)
    if conv.origin == None:
        conv.origin = fen_data_ll[0]
    fen_data_met = []
    for i in range(len(fen_data_ll)):
        fen_data_met.append([conv.degreesLatToMeters(fen_data_ll[i][0]), conv.degreesLongToMeters(fen_data_ll[i][1])])
    return fen_data_met
    f.close()

def conv_meters_graphics(p, top_left, zoom_rat, border):
    return [(p[0] - top_left[0]) * -zoom_rat + border, (p[1] - top_left[1]) * -zoom_rat + border]

#0 is border, 1 is zoom ratio
def graphical_consts(bounds, map):
    width = abs(bounds[1][0] - bounds[0][0])
    height = abs(bounds[1][1] - bounds[0][1])
    top_left = bounds[0]
    c_w = map.winfo_width()
    c_h = map.winfo_height()
    border = c_w * 0.04 if c_w <= c_h else c_h * 0.04#how far in to draw map
    w_zoom_rat = (c_w - (border * 2)) / width
    h_zoom_rat = (c_h - (border * 2)) / height
    zoom_rat = w_zoom_rat if w_zoom_rat <= h_zoom_rat else h_zoom_rat
    return [border, zoom_rat]
