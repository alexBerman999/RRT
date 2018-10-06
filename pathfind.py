import math
import random
import numpy
import maputil
import gwpputil

STEP = 10

class RRT_Node():
    def __init__(self, point, parent):
        self.point = point
        self.parent = parent

    def cost(self):
        if self.parent:
            return self.parent.cost() + dist(self.point, self.parent.point)
        return 0


def pathfind(start, end, mission):
    if valid_line(start, end, mission):
        return [start, end]
    return clean_rrt_star(start, end, mission)

#Distance between two points
def dist(p1,p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

def dist2d(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

def in_bounds(point, bounds):
    verts = bounds.vertices
    collisions = 0
    xPrime = point.x + 10000
    yPrime = point.y + 10000
    for i in range(1, len(verts) + 1):
        a = verts[i - 1].x
        b = verts[i - 1].y
        c = verts[i % len(verts)].x
        d = verts[i % len(verts)].y
        slope_orig = (yPrime - point.y) / (xPrime - point.x)
        if c == a:
            slope_edge = 1
        else:
            slope_edge = (d - b)/(c - a)
        if slope_orig != slope_edge:
            f = point.y - (slope_orig * point.x)
            g = b - (slope_edge * a)
            x_intersection = (g - f) / (slope_orig - slope_edge)
            y_intersection = (slope_orig * x_intersection) + f
            if x_intersection >= min(point.x, xPrime) and x_intersection <= max(point.x, xPrime):
                if y_intersection >= min(point.y, yPrime) and y_intersection <= max(point.y, yPrime):
                    if x_intersection >= min(a, c) and x_intersection <= max(a, c):
                        if y_intersection >= min(b, d) and y_intersection <= max(b, d):
                            collisions = collisions + 1
    return collisions % 2 == 1

def valid_point(p, mission):
    buffer = STEP * 1.5
    if not in_bounds(p, mission.bounds):
        return False
    for o in mission.static_obstacles:
        if p.z <= o.point.z and dist2d(p, o.point) <= (o.radius + buffer):
            return False
    return True

def valid_line(start, end, mission):
    #break line into points at delta apart
    delta = 1
    line_vector = [end.x - start.x, end.y - start.y, end.z - start.z]
    line_length = dist(start, end)
    x_comp = float(line_vector[0])/float(line_length)
    y_comp = float(line_vector[1])/float(line_length)
    z_comp = float(line_vector[2])/float(line_length)
    cur_point = maputil.Point(start.x, start.y, start.z)
    for i in range(int(line_length/delta)):
        cur_point.x = cur_point.x + (x_comp * delta)
        cur_point.y = cur_point.y + (y_comp * delta)
        cur_point.z = cur_point.z + (z_comp * delta)
        if not valid_point(cur_point, mission):
            return False
    return True

def valid_angle(parent_parent, parent, current):
    good_angle = 60 #degrees
    d1 = dist(parent_parent, parent)
    d2 = dist(parent, current)
    vector1 = [parent.x - parent_parent.x, parent.y - parent_parent.y, parent.z - parent_parent.z]
    vector2 = [current.x - parent.x, current.y - parent.y, current.z - parent.z]
    angle = math.degrees(math.acos(numpy.dot(vector1, vector2)/(d1 * d2)))
    if abs(angle) < good_angle:
        return True
    return False

def steer(node):
    if node.parent == None:
        return
    if dist(node.parent.point, node.point) > STEP:
        start = node.parent.point
        end = node.point
        line_vector = [end.x - start.x, end.y - start.y, end.z - start.z]
        line_length = dist(start, end)
        for i in range(len(line_vector)):
            line_vector[i] = line_vector[i]/line_length
        offset = [line_vector[0] * STEP, line_vector[1] * STEP, line_vector[2] * STEP]
        node.point.x = node.parent.point.x + offset[0]
        node.point.y = node.parent.point.y + offset[1]
        node.point.z = node.parent.point.z + offset[2]

def clean_path(path, mission):
    #remove unnecesary end nodes
    #by finding the first node that can directly connect to the end
    i = len(path) - 2
    earliest = i
    while i >= 0:
        if valid_line(path[i], path[-1], mission) and (i == 0 or valid_angle(path[i - 1], path[i], path[-1])):
            earliest = i
        i = i - 1

    path = path[:earliest + 1] + path[-1:]
    if len(path) < 3:
        return path
    i = 0
    j = 2
    while i < (len(path) - 2) and j < (len(path) - 1):
        if valid_line(path[i], path[j], mission):
            path = path[:i+1] + path[j:]
        else:
            i = i + 1
            j = j + 1
    return path

def rrt(start, end, mission):
    XDIM = abs(start.x - end.x) * 8
    YDIM = abs(start.y - end.y) * 8
    ZDIM = abs(start.z - end.z) * 8
    start_node = RRT_Node(start, None)
    tree = [start_node]
    end_node = RRT_Node(end, None)
    done = False
    while not done:
        #generate new point
        offset = [random.random() * XDIM, random.random() * YDIM, random.random() * ZDIM]
        offset[0] = offset[0] - (XDIM/2)
        offset[1] = offset[1] - (YDIM/2)
        offset[2] = offset[2] - (ZDIM/2)
        new_point = maputil.Point(start.x + offset[0], start.y + offset[1], start.z + offset[2])
        #Check to find closest point and steer
        closest = tree[0]
        for i in range(1, len(tree)):
            if dist(new_point, tree[i].point) < dist(new_point, closest.point):
                closest = tree[i]
        new_node = RRT_Node(new_point, closest)
        steer(new_node)
        #If the line from the parent to this node is valid, add it
        if valid_line(new_node.parent.point, new_node.point, mission) and ((not new_node.parent.parent) or valid_angle(new_node.parent.parent.point, new_node.parent.point, new_node.point)):
            tree.append(new_node)
            #Check to see if the path can be finished
            if valid_line(new_node.point, end, mission) and valid_angle(new_node.parent.point, new_node.point, end):
                end_node.parent = new_node
                done = True
    #follow chain back
    ptr = end_node
    path = []
    while ptr:
        path.append(ptr.point)
        ptr = ptr.parent
    path.reverse()

    print("Path Length: " + str(len(path)) + " Nodes")
    for i in range(len(path)):
        p = path[i]
        print("\t(" + str(p.x) + ",\t" + str(p.y) + ",\t" + str(p.z) + ")")
    return path

def rrt_star(start, end, mission):
    MIN_ITERATIONS = 1000
    MAX_ITERATIONS = 100000
    XDIM = abs(start.x - end.x) * 8
    YDIM = abs(start.y - end.y) * 8
    ZDIM = abs(start.z - end.z) * 8
    start_node = RRT_Node(start, None)
    tree = [start_node]
    end_node = RRT_Node(end, None)
    done = False
    iterations = 0
    while (not done) or iterations <= MIN_ITERATIONS:
        iterations = iterations + 1
        #In case no path is found in MAX_ITER iterations, just return a straight line
        if iterations >= MAX_ITERATIONS:
            print("**NO PATH FOUND**")
            return [start, end]
        #generate new point
        offset = [random.random() * XDIM, random.random() * YDIM, random.random() * ZDIM]
        offset[0] = offset[0] - (XDIM/2)
        offset[1] = offset[1] - (YDIM/2)
        offset[2] = offset[2] - (ZDIM/2)
        new_point = maputil.Point(start.x + offset[0], start.y + offset[1], start.z + offset[2])
        #Check to find closest point and steer
        closest = tree[0]
        for i in range(1, len(tree)):
            if dist(new_point, tree[i].point) < dist(new_point, closest.point):
                closest = tree[i]
        new_node = RRT_Node(new_point, closest)
        steer(new_node)
        #If the line from the parent to this node is valid, add it
        if valid_line(new_node.parent.point, new_node.point, mission) and ((not new_node.parent.parent) or valid_angle(new_node.parent.parent.point, new_node.parent.point, new_node.point)):
            tree.append(new_node)
            #Look for other nodes within the STEP range and rearrange tree
            close_nodes = []
            for n in tree:
                if n != new_node and dist(new_node.point, n.point) <= STEP:
                    close_nodes.append(n)
            for n in close_nodes:
                if n.parent and new_node.cost() < n.parent.cost():
                    n.parent = new_node
            #Check to see if the path can be finished
            if valid_line(new_node.point, end, mission) and valid_angle(new_node.parent.point, new_node.point, end):
                end_node.parent = new_node
                done = True
    #follow chain back
    ptr = end_node
    path = []
    while ptr:
        path.append(ptr.point)
        ptr = ptr.parent
    path.reverse()

    print("Path Length: " + str(len(path)) + " Nodes")
    for i in range(len(path)):
        p = path[i]
        print("\t(" + str(p.x) + ",\t" + str(p.y) + ",\t" + str(p.z) + ")")
    return path

def clean_rrt_star(start, end, mission):
    path = rrt_star(start, end, mission)
    print(len(path))
    path = clean_path(path, mission)
    print(len(path))
    return path
