class Point():
    def __init__(self, x, y, z = 0):
        self.x = x
        self.y = y
        self.z = z
    def __str__(self):
        return str(self.x) + "\t" + str(self.y) + "\t" + str(self.z)

class Obstacle():
    def __init__(self, point, radius):
        self.point = point
        self.radius = radius

class Fence():
    def __init__(self, points):
        self.vertices = points

    def point_in_fence(point):
        collisions = 0
        x,y = point.x, point.y
        xPrime = x + 10000
        yPrime = y + 10000
        for i in range(1, len(poly) + 1):
            a = self.vertices[i - 1].x
            b = self.vertices[i - 1].y
            c = self.vertices[i % len(poly)].x
            d = self.vertices[i % len(poly)].y
            slope_orig = (yPrime - y) / (xPrime - x)
            if c == a:
                slope_edge = 1
            else:
                slope_edge = (d - b)/(c - a)
            if slope_orig != slope_edge:
                f = y - (slope_orig * x)
                g = b - (slope_edge * a)
                x_intersection = (g - f) / (slope_orig - slope_edge)
                y_intersection = (slope_orig * x_intersection) + f
                if x_intersection >= min(x, xPrime) and x_intersection <= max(x, xPrime):
                    if y_intersection >= min(y, yPrime) and y_intersection <= max(y, yPrime):
                        if x_intersection >= min(a, c) and x_intersection <= max(a, c):
                            if y_intersection >= min(b, d) and y_intersection <= max(b, d):
                                collisions = collisions + 1
            return collisions % 2 == 1
