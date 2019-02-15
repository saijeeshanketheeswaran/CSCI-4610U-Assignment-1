'''
A* search

 Saijeeshan Ketheeswaran, Abinash Narendiran, Rohil Arya
'''
from tkinter import *
import struct
import xml.etree.ElementTree as ET
from queue import *
import math

MINLON = -78.8685000
MAXLON = -78.8150000
MAXLAT = 43.9262000
MINLAT = 43.9066000

WIDTH = MAXLON - MINLON
HEIGHT = MAXLAT - MINLAT

LONRAT = math.cos(MAXLAT * 3.1415 / 180)

WINWIDTH = 1000
WINHEIGHT = (int)((WINWIDTH / LONRAT) * HEIGHT / WIDTH)

XPIX = WINWIDTH / WIDTH
YPIX = WINHEIGHT / HEIGHT

# width,height of elevation array
elev_pix = 3601

# some constants about the earth
MPERLAT = 111000 # meters per degree of latitude, approximately
MPERLON = MPERLAT * LONRAT # meters per degree longitude

def n_distance(n1, n2):
    dist_x = (n2.position[0] - n1.position[0]) * MPERLON
    dist_y = (n2.position[1] - n1.position[1]) * MPERLAT
    return math.sqrt(dist_x * dist_x + dist_y * dist_y)


def elev_distance(n1, n2):
    return n2.elevation - n1.elevation


class Node():
    __slots__ = ('id', 'position', 'ways', 'elevation', 'str_way', 'set_way')

    def __init__(self, id, pos, elev=0):
        self.id = id
        self.str_way = None
        self.position = pos
        self.ways = []
        self.elevation = elev

    def __str__(self):
        if self.str_way is None:
            self.str_way = self.get_waystr()

        return str(self.position) + ": " + self.str_way

    def get_waystr(self):
        if self.str_way is None:
            self.str_way = ""
            self.set_way = set()

            for w in self.ways:
                self.set_way.add(w.way.name)

        return self.str_way


class Edge():
    __slots__ = ('way', 'destination', 'cost')

    def __init__(self, w, src, dest):
        self.way = w
        self.destination = dest
        self.cost = n_distance(src, dest)

        if dest.elevation > src.elevation:
            self.cost += (dest.elevation - src.elevation) * 2

            if self.way.type == 'steps':
                self.cost *= 1.5


class Way():
    __slots__ = ('name', 'type', 'nodes')

    def __init__(self, n, t):
        self.name = n
        self.type = t
        self.nodes = []


class Planner():
    __slots__ = ('nodes', 'ways')

    def __init__(self, n, w):
        self.nodes = n
        self.ways = w

    def heuristic(self, node, gnode):

        return math.sqrt(elev_distance(node, gnode)**2 + n_distance(node, gnode)**2)

    def plan(self, s, g):

        parents = {}
        costs = {}

        queue = PriorityQueue()
        queue.put((self.heuristic(s, g), s))

        parents[s] = None
        costs[s] = 0

        while not queue.empty():
            cf, cnode = queue.get()

            if cnode == g:
                print("Found a path, and time is ", costs[g] * 60 / 5000, "\n")
                return self.make_path(parents, g)

            for edge in cnode.ways:
                new_cost = costs[cnode] + edge.cost

                if edge.destination not in parents or new_cost < costs[edge.destination]:
                    parents[edge.destination] = (cnode, edge.way)
                    costs[edge.destination] = new_cost
                    queue.put((self.heuristic(edge.destination, g) + new_cost, edge.destination))

    def make_path(self, par, g):
        nodes = []
        ways = []
        current = g
        nodes.append(current)

        while par[current] is not None:
            previous, way = par[current]
            ways.append(way.name)
            nodes.append(previous)
            current = previous

        nodes.reverse()
        ways.reverse()

        return nodes, ways


class PlanWin(Frame):

    __slots__ = ('whatis', 'nodes', 'ways', 'elevs', 'nodelab', 'elab', 'planner', 'lastnode', 'start_node', 'target_node')

    def lat_lon_to_pix(self, lat_long):

        x = (lat_long[1] - MINLON) * (XPIX)
        y = (MAXLAT - lat_long[0]) * (YPIX)

        return x, y

    def pix_to_elev(self, x, y):

        return self.lat_lon_to_elev(((MAXLAT - (y / YPIX)), ((x / XPIX) + MINLON)))

    def lat_lon_to_elev(self, lat_long):

        row = (int)((43.9682000 - lat_long[0]) * elev_pix)
        col = (int)((lat_long[1] - 78.9425000) * elev_pix)

        return self.elevs[row * elev_pix + col]

    def hover_map(self, event):

        self.elab.configure(text=str(self.pix_to_elev(event.x, event.y)))

        for (dist_x, dist_y) in [(0, 0), (-1, 0), (0, -1), (1, 0), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:

            ckpos = (event.x + dist_x, event.y + dist_y)

            if ckpos in self.whatis:

                self.lastnode = self.whatis[ckpos]
                lnpos = self.lat_lon_to_pix(self.nodes[self.lastnode].position)
                self.canvas.coords('lastdot', (lnpos[0] - 2, lnpos[1] - 2, lnpos[0] + 2, lnpos[1] + 2))
                nstr = str(self.lastnode)
                nstr += " "
                nstr += str(self.nodes[self.whatis[ckpos]].get_waystr())
                self.nodelab.configure(text=nstr)

                return

    def mapclick(self, event):
        if self.lastnode is None:
            return

        if self.start_node is None:
            self.start_node = self.nodes[self.lastnode]
            self.snpix = self.lat_lon_to_pix(self.start_node.position)
            self.canvas.coords('greendot', (self.snpix[0] - 2, self.snpix[1] - 2, self.snpix[0] + 2, self.snpix[1] + 2))

        elif self.target_node is None:
            self.target_node = self.nodes[self.lastnode]
            self.snpix = self.lat_lon_to_pix(self.target_node.position)
            self.canvas.coords('reddot', (self.snpix[0] - 2, self.snpix[1] - 2, self.snpix[0] + 2, self.snpix[1] + 2))

    def clear(self):
        ''' Clear button callback. '''
        self.lastnode = None
        self.target_node = None
        self.start_node = None
        self.canvas.coords('greendot', (0, 0, 0, 0))
        self.canvas.coords('reddot', (0, 0, 0, 0))
        self.canvas.coords('path', (0, 0, 0, 0))

    def plan_path(self):

        print("Planning a path... \n")

        if self.start_node is None or self.target_node is None:

            print("Sorry, not enough info.")

            return

        print("From start node", self.start_node.id, "to target node", self.target_node.id, "\n")

        nodes, ways = self.planner.plan(self.start_node, self.target_node)
        prev_way = ""

        for name_of_way in ways:

            if name_of_way != prev_way:
                print(name_of_way)
                prev_way = name_of_way

        coords = []

        for node in nodes:

            num_pos = self.lat_lon_to_pix(node.position)
            coords.append(num_pos[0])
            coords.append(num_pos[1])

        self.canvas.coords('path', *coords)

    def __init__(self, master, nodes, ways, coast_nodes, elevs):
        self.whatis = {}
        self.nodes = nodes
        self.ways = ways
        self.elevs = elevs
        self.start_node = None
        self.target_node = None
        self.planner = Planner(nodes, ways)

        thewin = Frame(master)

        w = Canvas(thewin, width = WINWIDTH, height = WINHEIGHT)
        w.bind("<Button-1>", self.mapclick)
        w.bind("<Motion>", self.hover_map)

        for waynum in self.ways:
            node_list = self.ways[waynum].nodes
            thispix = self.lat_lon_to_pix(self.nodes[node_list[0]].position)

            if len(self.nodes[node_list[0]].ways) > 2:
                self.whatis[((int)(thispix[0]), (int)(thispix[1]))] = node_list[0]

            for n in range(len(node_list) - 1):
                nextpix = self.lat_lon_to_pix(self.nodes[node_list[n + 1]].position)
                self.whatis[((int)(nextpix[0]), (int)(nextpix[1]))] = node_list[n + 1]
                w.create_line(thispix[0], thispix[1], nextpix[0], nextpix[1])
                thispix = nextpix

        if len(coast_nodes):
            thispix = self.lat_lon_to_pix(self.nodes[coast_nodes[0]].position)

        for n in range(len(coast_nodes) - 1):
            nextpix = self.lat_lon_to_pix(self.nodes[coast_nodes[n + 1]].position)
            w.create_line(thispix[0], thispix[1], nextpix[0], nextpix[1], fill="blue")
            thispix = nextpix

        w.create_line(0, 0, 0, 0, fill='yellow', width=3, tag='path')

        w.create_oval(0, 0, 0, 0, outline='green', fill='green', tag='greendot')
        w.create_oval(0, 0, 0, 0, outline='red', fill='red', tag='reddot')
        w.create_oval(0, 0, 0, 0, outline='blue', fill='blue', tag='lastdot')

        w.pack(fill=BOTH)

        self.canvas = w

        cb = Button(thewin, text="Clear Plan", command=self.clear)
        cb.pack(side=RIGHT, pady=5)

        sb = Button(thewin, text="Show Plan", command=self.plan_path)
        sb.pack(side=RIGHT, pady=5)

        nodelablab = Label(thewin, text="Node:")
        nodelablab.pack(side=LEFT, padx=5)

        self.nodelab = Label(thewin, text="None")
        self.nodelab.pack(side=LEFT, padx=5)

        elablab = Label(thewin, text="Elevation:")
        elablab.pack(side=LEFT, padx=5)

        self.elab = Label(thewin, text="0")
        self.elab.pack(side=LEFT, padx=5)

        thewin.pack()


def read_elevations(efilename):

    efile = open(efilename, "rb")
    estr = efile.read()
    elevs = []

    for spot in range(0, len(estr), 2):

        elevs.append(struct.unpack('<h', estr[spot:spot + 2])[0])

    return elevs


def read_xml(elevs):
    ''' Build the search graph from the OpenStreetMap XML. '''
    tree = ET.parse('Oshawa.osm')
    root = tree.getroot()

    nodes = dict()
    ways = dict()
    waytypes = set()
    coast_nodes = []

    for item in root:

        if item.tag == 'node':
            coords = ((float)(item.get('lat')), (float)(item.get('lon')))

            elev_row = (int)((43 - coords[0]) * elev_pix)

            elev_col = (int)((coords[1] - 18) * elev_pix)
            try:
                el = elevs[elev_row * elev_pix + elev_col]
            except IndexError:
                el = 0
            nodes[(int)(item.get('id'))] = Node((int)(item.get('id')), coords, el)

        elif item.tag == 'way':

            if item.get('id') == '157161112':  # main coastline way ID

                for a in item:
                    if a.tag == 'nd':
                        coast_nodes.append((int)(a.get('ref')))

                continue

            use_path = False
            one_way_path = False
            path_name = 'unnamed way'

            for a in item:

                if a.tag == 'tag' and a.get('k') == 'highway':
                    use_path = True
                    mytype = a.get('v')

                if a.tag == 'tag' and a.get('k') == 'name':
                    path_name = a.get('v')

                if a.tag == 'tag' and a.get('k') == 'one_way_path':
                    if a.get('v') == 'yes':
                        one_way_path = True

            if use_path:
                path_id = (int)(item.get('id'))
                ways[path_id] = Way(path_name, mytype)
                node_list = []

                for a in item:
                    if a.tag == 'nd':
                        node_list.append((int)(a.get('ref')))

                this_node = node_list[0]

                for n in range(len(node_list) - 1):
                    next_node = node_list[n + 1]
                    nodes[this_node].ways.append(Edge(ways[path_id], nodes[this_node], nodes[next_node]))
                    this_node = next_node

                if not one_way_path:
                    this_node = node_list[-1]

                    for n in range(len(node_list) - 2, -1, -1):
                        next_node = node_list[n]
                        nodes[this_node].ways.append(Edge(ways[path_id], nodes[this_node], nodes[next_node]))
                        this_node = next_node

                ways[path_id].nodes = node_list

    print(len(coast_nodes))

    if len(coast_nodes):
        print(coast_nodes[0])
        print(nodes[coast_nodes[0]])

    return nodes, ways, coast_nodes

elevs = read_elevations("n41_w114_1arc_v2_bil/n41_w114_1arc_v2.bil")
nodes, ways, coast_nodes = read_xml(elevs)

master = Tk()
thewin = PlanWin(master, nodes, ways, coast_nodes, elevs)
mainloop()