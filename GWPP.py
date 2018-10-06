from tkinter import *
import gwpputil
import maputil

class Application(Frame):
    def __init__(self, master = None):
        super().__init__(master)
        self.pack()
        self.v = IntVar()
        self.v.set(0)
        self.create_widgets()
        self.conv = gwpputil.Deg_Met_Converter()
        self.mission = None

    def create_widgets(self):
        #Set Mission Parameters Button
        self.set_mission_param_b = Button(self)
        self.set_mission_param_b["text"] = "Set Mission Parameters"
        self.set_mission_param_b["command"] = self.set_mission_param
        self.set_mission_param_b.pack(side = "top")

        #Generate Flightplan Button
        self.generate_flightplan_b = Button(self)
        self.generate_flightplan_b["text"] = "Generate Flightplan"
        self.generate_flightplan_b["command"] = self.generate_flightplan
        self.generate_flightplan_b.pack(side = "top")

        #Live Telemetry Radiobutton
        self.live_telem = Radiobutton(self)
        self.live_telem["text"] = "Live Telemetry"
        self.live_telem["variable"] = self.v
        self.live_telem["value"] = 0
        self.live_telem["command"] = self.switch_telem_mode
        self.live_telem.pack(side = "bottom")

        #Telemetry From File Radiobutton
        self.test_telem = Radiobutton(self)
        self.test_telem["text"] = "Telemetry From File"
        self.test_telem["variable"] = self.v
        self.test_telem["value"] = 1
        self.test_telem["command"] = self.switch_telem_mode
        self.test_telem.pack(side = "bottom")

        #Canvas
        self.canv_default_width = 600
        self.canv_default_height = 600
        self.map = Canvas(self, width = self.canv_default_width, height = self.canv_default_height)
        self.map.pack(side = "right")
        self.draw_background("BLACK")

    def set_mission_param(self):
        fen = maputil.Fence(gwpputil.read_lat_long("file/fence.txt", self.conv))
        way = gwpputil.read_lat_long("file/wp.txt", self.conv)
        search = maputil.Fence(gwpputil.read_lat_long("file/search.txt", self.conv))
        stat_obsts = gwpputil.read_obst_file("file/obst.txt", self.conv)
        self.mission = gwpputil.Mission_Param(fen, way, search, stat_obsts)
        self.redraw_map()
        print("Setting mission parameters...")


    def generate_flightplan(self):
        if self.mission == None:
            return
        self.mission.generate_flightplan(maputil.Point(0, 0), 270)
        self.redraw_map()
        print("Generating Flightplan...")

    def switch_telem_mode(self):
        type = self.v.get()
        if type == 0:
            print("Live Telemetry")
        elif type == 1:
            print("Test Telemetry")

    def redraw_map(self):
        self.map.delete("all")
        self.draw_background("BLACK")

        if self.mission == None:
            return
        bounds = self.mission.bounding_rect()
        border, zoom_rat = gwpputil.graphical_consts(bounds, self.map)
        rel_orig = bounds[0]

        self.draw_border(self.mission.bounds, rel_orig, zoom_rat, border, "YELLOW")
        self.draw_border(self.mission.search_area, rel_orig, zoom_rat, border, "BLUE")
        for obstacle in self.mission.static_obstacles:
            self.draw_circle(obstacle.point, obstacle.radius * zoom_rat, rel_orig, zoom_rat, border, "RED")
        for point in self.mission.waypoints:
            self.draw_circle(point, 5, rel_orig, zoom_rat, border, "GREEN")
        if self.mission.flightplan != None:
            fp = self.mission.flightplan
            for i in range(1, len(fp)):
                p = fp[i - 1]
                q = fp[i]
                self.draw_line(p, q, rel_orig, zoom_rat, border, "WHITE")
                self.draw_circle(p, 2, rel_orig, zoom_rat, border, "WHITE")
        #draw search srch_area
        print("Redrawing Map")

    def draw_border(self, fence, rel_orig, zoom_rat, border, color):
        for i in range(1, len(fence.vertices) + 1):
            p = fence.vertices[i - 1]
            q = fence.vertices[i % (len(fence.vertices))]
            self.draw_line(p, q, rel_orig, zoom_rat, border, color)

    def draw_line(self, p, q, rel_orig, zoom_rat, border, color):
        p = gwpputil.conv_meters_graphics([p.x, p.y], rel_orig, zoom_rat, border)
        q = gwpputil.conv_meters_graphics([q.x, q.y], rel_orig, zoom_rat, border)
        self.map.create_line(p[0], p[1], q[0], q[1], fill = color)

    def draw_circle(self, p, radius, rel_orig, zoom_rat, border, color):
        p = gwpputil.conv_meters_graphics([p.x, p.y], rel_orig, zoom_rat, border)
        radius = radius
        self.map.create_oval(p[0] - radius, p[1] - radius, p[0] + radius, p[1] + radius, fill = color)

    def draw_background(self, color):
        self.map.create_rectangle(0, 0, self.canv_default_width, self.canv_default_height, fill = color)

root = Tk()
gwpp = Application(master = root)
gwpp.mainloop()
