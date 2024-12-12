import copy
import numpy as np
import heapq
import itertools
import math
import sys
import time
from statistics import median, mean

from croblink import *
from math import *
import xml.etree.ElementTree as ET
import collections
import pprint
from enum import Enum
from collections import deque

pp = pprint.PrettyPrinter(indent=10)

CELLROWS = 7
CELLCOLS = 14


class orientation(Enum):
    Right = 0
    Up = 1
    Left = 2
    Down = 3


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, filename, filename_pathing):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.rob_name = rob_name
        self.current_measures = []
        self.state = "map"
        self.possible_headings = [0, 90, 180, 270]
        self.previous_x = 0
        self.previous_y = 0
        self.orientation = orientation.Right
        self.first_boot = True
        self.moving = False
        self.rotating = False
        self.mymap = np.full(((CELLROWS * 4 - 1), (CELLCOLS * 4 - 1)), ' ', dtype=str)  # Fill map with empty spaces initially [[' '] * (CELLCOLS * 4 - 1) for i in range(CELLROWS * 4 - 1)]
        self.map_location_x = 27
        self.map_location_y = 13
        self.mymap[13][27] = "I"
        self.visited = set()
        self.not_visited = set()
        self.target_locked = None
        self.target_location = None
        self.graph = {}
        self.path = None
        self.flag = False
        self.need_to_rotate = False
        self.filename = filename
        self.dead_end = []
        self.right_power = 0
        self.left_power = 0
        self.estimated_x = 0
        self.estimated_y = 0
        self.heading = 0
        self.estimated = 0
        self.estimated_degrees = 0
        self.heading_buffer = collections.deque(maxlen=3)
        self.stop_counter = 0
        self.measure_counter = 0
        self.previous_state = "map"
        self.supposed_x = 0
        self.supposed_y = 0
        self.next_state = None
        self.right_power = 0
        self.left_power = 0
        self.go_back = False
        self.beacons_positions = None
        self.filename_pathing = filename_pathing
        self.shortest_path = None


    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        self.beacons_positions = [(-1, -1) for i in range(int(self.nBeacons))]
        while True:

            self.previous_state = self.state
            self.readSensors()


            if self.first_boot:
                self.previous_x = self.estimated_x
                self.previous_y = self.estimated_y
                self.first_boot = False
                offset_x = self.measures.x
                offset_y = self.measures.y


            print("ROBOT GPS {},{}".format(self.measures.x-offset_x, self.measures.y-offset_y))
            print("Estimated Position {},{}".format(self.estimated_x, self.estimated_y))

            if self.measures.compass < 0:
                self.measures.compass = self.measures.compass + 360

            self.heading_buffer.append(self.measures.compass)

            self.current_measures = [self.measures.irSensor[0], self.measures.irSensor[1],
                                     self.measures.irSensor[2], self.measures.irSensor[3]]

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed == True:
                    state = 'wait'

                if self.measures.ground == 0:
                    self.setVisitingLed(True);
                self.wander()
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    state = 'return'
                self.driveMotors(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    self.setReturningLed(False)
                self.wander()

    def wander(self):
        if self.state == "map":
            if self.measures.ground != -1:
                # saving the beacon location
                self.beacons_positions[self.measures.ground] = (self.map_location_x, self.map_location_y)
            #print("I am mapping")
            self.map()
            if self.path is not None and self.path != []:
                self.state = "go_with_purpose"
            else:
                self.state = "go"

        elif self.state == "update_gps":
            center, left, right = self.current_measures[:3]
            center_distance, left_distance, right_distance = 1 / center, 1 / left, 1 / right
            self.state, self.next_state = self.next_state, None
            
            print("UPDATED GPS")
            if self.orientation == orientation.Right:
                if center > 1.2:
                    wall_pos = self.supposed_x + 1
                    self.estimated_x = wall_pos - center_distance - 0.5
                if right > 1.2:
                    wall_pos = self.supposed_y - 1
                    self.estimated_y = wall_pos + right_distance + 0.5
            elif self.orientation == orientation.Left:
                if center > 1.2:
                    wall_pos = self.supposed_x - 1
                    self.estimated_x = wall_pos + center_distance + 0.5
                if right > 1.2:
                    wall_pos = self.supposed_y + 1
                    self.estimated_y = wall_pos - right_distance - 0.5
            elif self.orientation == orientation.Up:
                if center > 1.2:
                    wall_pos = self.supposed_y + 1
                    self.estimated_y = wall_pos - center_distance - 0.5
                if right > 1.2:
                    wall_pos = self.supposed_x + 1
                    self.estimated_x = wall_pos - right_distance - 0.5
            elif self.orientation == orientation.Down:
                if center > 1.2:
                    wall_pos = self.supposed_y - 1
                    self.estimated_y = wall_pos + center_distance + 0.5
                if right > 1.2:
                    wall_pos = self.supposed_x - 1
                    self.estimated_x = wall_pos + right_distance + 0.5
        
        elif self.state == "go_with_purpose":

            if self.target_location is None:
                # print("where_to_advanced")
                self.where_to_advanced()
                if self.path is None or self.path == []:
                    return
            if not self.need_to_rotate:
                temp_orientation = self.where_to_basic(self.path[0])
                if temp_orientation != self.orientation:
                    self.target_locked = temp_orientation
                    self.need_to_rotate = True
            if self.need_to_rotate:
                self.rotate()
                return
            self.go()

        elif self.state == "go":
            # print("I am going")
            #print(self.map_location_x, self.map_location_y)
            if self.next():
                self.go()
            else:
                #print("No path forward")
                self.where_to_basic()

        elif self.state == "rotate":
            # print("rotate")
            self.rotate()

        elif self.state == "stop":
            #print("I am stopping")
            if self.target_location == (self.map_location_x, self.map_location_y):
                #print("clearing path")
                self.path = None
                self.target_location = None
                self.target_locked = None
            if self.path is None or self.path == []:
                self.stop()
                self.next_state = "map"
                self.state = "update_gps"
                self.stop_counter = 0
            else:
                self.next_state = "go_with_purpose"
                self.state = "update_gps"
                self.slowdown()
                self.stop_counter = 0
            # self.state = "stop"
        elif self.state == "end":
            if self.go_back:
                self.finish()
                self.create_mapping_file()
                self.calculate_beacon_paths()
                self.create_pathing_file()
            self.not_visited.add((27,13))
            self.state = "go_with_purpose"
            self.target_location = None
            self.visited.remove((27,13))
            self.go_back = True




    def where_to_basic(self, target=None):
        # Are there any free, not visited, spaces adjacent to me
        level = 2
        wall_level = 1
        #print(target)

        possible_places = [((self.map_location_x - level, self.map_location_y), orientation.Left,
                            (self.map_location_x - wall_level, self.map_location_y)),
                           ((self.map_location_x + level, self.map_location_y), orientation.Right,
                            (self.map_location_x + wall_level, self.map_location_y)),
                           ((self.map_location_x, self.map_location_y - level), orientation.Up,
                            (self.map_location_x, self.map_location_y - wall_level)),
                           ((self.map_location_x, self.map_location_y + level), orientation.Down,
                            (self.map_location_x, self.map_location_y + wall_level))]
        for adjacent in possible_places:
            # print(adjacent)
            # print(self.mymap[adjacent[2][1]][adjacent[2][0]])
            try:
                if target is not None:
                    if adjacent[0] == target:
                        return adjacent[1]
                else:
                    if adjacent[0] not in self.visited and self.mymap[adjacent[0][1]][adjacent[0][0]] == 'X' \
                            and self.mymap[adjacent[2][1]][adjacent[2][0]] != "|" and self.mymap[adjacent[2][1]][
                        adjacent[2][0]] != "-":
                        self.target_locked = adjacent[1]
                        #print(self.target_locked)
                        self.state = "rotate"
                        #print("found path now i am rotating")
                        return
            except IndexError as e:
                continue
        if self.target_locked is None:
            self.state = "go_with_purpose"

    def where_to_advanced(self):
        min = 99

        for key in self.graph.keys():
            self.graph[key] = list(set(self.graph[key]))

        for node in self.not_visited:
            self.graph.setdefault(node, [])

        self.not_visited = self.not_visited - self.visited
        for node in self.not_visited:
            _, path = self.dijkstra(self.graph, (self.map_location_x, self.map_location_y))
            path = list(self.get_path((self.map_location_x, self.map_location_y), node, path))
            if len(path) < min:
                self.target_location = node
                self.path = path
                min = len(path)

        if self.target_location is None:
            self.state = "end"
            return

    def rotate(self):
        target_heading = self.possible_headings[self.target_locked.value]

        # print(target_heading, self.measures.compass, self.target_locked)
        factor = self.get_rotation_factor(soft_rotation=False, target=target_heading)
        # print(factor)
        if not (target_heading - 2 <= median(self.heading_buffer) <= target_heading + 2):
            print("{},{}".format(target_heading, self.estimated))
            self.move(0, 0.025, 0, factor, rotate=True)
            #self.simple_rotate(0.04)
        else:
            self.heading = target_heading
            if self.target_location is None:
                self.state = "go"
            else:
                self.state = "go_with_purpose"
                self.need_to_rotate = False
            self.orientation = self.target_locked
            self.target_locked = None

    def go(self):
        inertia_comp = 1.65 if self.state == "go_with_purpose" else 1.74
        speed = 0.15
        factor = self.get_rotation_factor()

        adjustments = {
            orientation.Right:  (self.estimated_x < self.supposed_x + inertia_comp, 2, 0, 2, 0),
            orientation.Left:   (self.estimated_x > self.supposed_x - inertia_comp, -2, 0, -2, 0),
            orientation.Up:     (self.estimated_y < self.supposed_y + inertia_comp, 0, 2, 0, -2),
            orientation.Down:   (self.estimated_y > self.supposed_y - inertia_comp, 0, -2, 0, 2),
        }

        move_check, dx, dy, mx, my = adjustments[self.orientation]

        if move_check:
            self.move(speed, 0.01, 0, factor)
        else:
            self.moving = False
            self.supposed_x += dx
            self.supposed_y += dy
            self.estimated_x = (self.estimated_x + self.supposed_x) / 2
            self.estimated_y = (self.estimated_y + self.supposed_y) / 2
            self.previous_x = self.estimated_x
            self.previous_y = self.estimated_y
            self.map_location_x += mx
            self.map_location_y += my
            self.state = "stop"
            self.visited.add((self.map_location_x, self.map_location_y))
            self.not_visited.discard((self.map_location_x, self.map_location_y))
            if self.path is not None:
                self.path = self.path[1:]
        return

    def get_rotation_factor(self, soft_rotation=True, target=None):
        supposed_heading = self.possible_headings[self.orientation.value] if soft_rotation else target
        real_heading = self.measures.compass
        diff = 360 - real_heading if real_heading > 180 and supposed_heading == 0 else abs(supposed_heading - real_heading)
        
        if (360 + supposed_heading - real_heading) % 360 > 180:
            if target is not None:
                return 0.15 * diff
            else:
                return 0.5 * diff
        else:
            if target is not None:
                return -0.15 * diff
            else:
                return -0.5 * diff

    def move(self, linear, k, m, r, rotate=False):
        self.right_power = linear + (k * (m - r)) / 2
        self.left_power = linear - (k * (m - r)) / 2
        if not rotate:
            self.update_self_gps(self.left_power, self.right_power)
        self.driveMotors(self.left_power, self.right_power)

    def update_self_gps(self, left, right):

        self.left_power = (0.5 * left + 0.5 * self.left_power)
        self.right_power = (0.5 * right + 0.5 * self.right_power)

        lin = (self.left_power + self.right_power) / 2
        rot = (self.right_power - self.left_power)

        self.estimated_x = self.estimated_x + lin * cos(radians(self.measures.compass))
        self.estimated_y = self.estimated_y + lin * sin(radians(self.measures.compass))
        self.estimated += rot
        self.estimated_degrees = degrees(self.estimated)
        print(self.estimated_degrees)

    def stop(self):
        self.driveMotors(0.0, 0.0)

    def map(self):
        
        DIRECTION_MAP = {
        orientation.Right: [(0, 1, "|"), (-1, 0, "-"), (1, 0, "-"), (0, -1, "|")],
        orientation.Up: [(-1, 0, "-"), (0, -1, "|"), (0, 1, "|"), (1, 0, "-")],
        orientation.Down: [(1, 0, "-"), (0, 1, "|"), (0, -1, "|"), (-1, 0, "-")],
        orientation.Left: [(0, -1, "|"), (1, 0, "-"), (-1, 0, "-"), (0, 1, "|")]
        }

        # Set current location as visited
        self.visited.add((self.map_location_x, self.map_location_y))

        # Get current direction offsets and symbols based on the robot's orientation
        direction_offsets = DIRECTION_MAP[self.orientation]

        # Iterate through the direction offsets and current measures
        for i, (dy, dx, symbol) in enumerate(direction_offsets):
            adj_y, adj_x = self.map_location_y + dy, self.map_location_x + dx
            adj_far_y, adj_far_x = self.map_location_y + 2 * dy, self.map_location_x + 2 * dx
            
            # Check the measure for each direction
            if self.current_measures[i] > 1:
                self.mymap[adj_y, adj_x] = symbol
            else:
                self.mymap[adj_y, adj_x] = "X"
                self.mymap[adj_far_y, adj_far_x] = "X"
                # Update graph and not_visited set
                self.graph.setdefault((self.map_location_x, self.map_location_y), []).append(((adj_far_x, adj_far_y), 1))
                self.not_visited.add((adj_far_x, adj_far_y))


        return


    def next(self):
        try:
            # Define the offsets and symbols based on orientation
            CHECK_MAP = {
                orientation.Right: ((0, 2), (0, 1), "|"),
                orientation.Up: ((-2, 0), (-1, 0), "-"),
                orientation.Left: ((0, -2), (0, -1), "|"),
                orientation.Down: ((2, 0), (1, 0), "-")
            }

            # Get the specific offset and symbol for the current orientation
            far_offset, near_offset, barrier_symbol = CHECK_MAP[self.orientation]

            # Calculate positions
            far_y, far_x = self.map_location_y + far_offset[0], self.map_location_x + far_offset[1]
            near_y, near_x = self.map_location_y + near_offset[0], self.map_location_x + near_offset[1]

            # Check the conditions in a single return statement
            return (self.mymap[far_y, far_x] == "X" and
                    self.mymap[near_y, near_x] != barrier_symbol and
                    (self.map_location_x + far_offset[1], self.map_location_y + far_offset[0]) not in self.visited)
        except IndexError:
            # Return False if any indexing errors occur
            return False

    def create_mapping_file(self):
        self.mymap[13][27] = "I"
        for i,beacon in enumerate(self.beacons_positions):
            self.mymap[beacon[1]][beacon[0]] = i
        f = open(self.filename,"w")

        for row in self.mymap:
            for val in row:
                f.write("{:1}".format(val))
            f.write("\n")

        f.close()

    def create_pathing_file(self):
        f = open(self.filename_pathing, "w")
        initial_pos_x = 27
        initial_pos_y = 13
        f.write("0 0\n")
        for node in self.shortest_path:
            x = node[0] - initial_pos_x
            y = initial_pos_y - node[1]
            f.write("{} {}\n".format(x, y))

    def dijkstra(self, graph, start):
        distances = {x: float('inf') for x in graph.keys()}
        previous = {x: None for x in graph.keys()}
        distances[start] = 0
        queue = [(start, 0)]
        while queue:
            node, distance = heapq.heappop(queue)
            for neighbor, cost in graph[node]:
                temp = distance + cost
                if temp < distances[neighbor]:
                    distances[neighbor] = temp
                    previous[neighbor] = node
                    heapq.heappush(queue, (neighbor, temp))
        return distances, previous

    def get_path(self, start, end, backtracking):
        path = deque()
        current = end
        while backtracking[current] is not None:
            path.appendleft(current)
            current = backtracking[current]
        return path

    def slowdown(self):
        self.update_self_gps(0.08,0.08)
        self.driveMotors(0.08,0.08)

    def calculate_beacon_paths(self):
        
        shortest = 99
        shortest_path = None
        shortest_permutation = None
        paths, perm_paths = {}, {}
        
        # # Ensure graph uniqueness and add unvisited nodes to graph
        for key in self.graph.keys():
            self.graph[key] = list(set(self.graph[key]))
        self.not_visited -= self.visited
        for node in self.not_visited:
            self.graph.setdefault(node, [])


        new_graph = copy.deepcopy(self.graph)
        for row in range(1, len(self.mymap), 2):
            for i, cell in enumerate(self.mymap[row]):
                possible_places = [((i + 2, row), 1, (i + 1, row)), ((i - 2, row), 1, (i - 1, row)),
                                   ((i, row + 2), 1, (i, row + 1)), ((i, row - 2), 1, (i, row - 1))]
                if cell == ' ':
                    for place in possible_places:
                        try:
                            if place[0][0] >= 0 and place[0][1] >= 0 and \
                            self.mymap[place[2][1]][place[2][0]] not in "|-":
                                new_graph.setdefault((i, row), []).append((place[0], 1))
                        except:
                            # print("except {}".format(place))
                            continue
                try:
                    if new_graph[(i, row)] == []:
                        for place in possible_places:
                            try:
                                if place[0][0] >= 0 and place[0][1] >= 0 and \
                                self.mymap[place[2][1]][place[2][0]] not in "|-":
                                    new_graph.setdefault((i, row), []).append((place[0], 1))
                            except:
                                # print("except {}".format(place))
                                continue
                except:
                    #print("except {}".format((i, row)))
                    continue

        # # Calculate Dijkstra paths for beacons in both known and unknown maps
        for beacon in self.beacons_positions:
            paths[beacon] = self.dijkstra(self.graph, beacon)[1]
            # paths_unk[beacon] = self.dijkstra(new_graph, beacon)[1]

        # # Generate closed path permutations starting from (27,13)
        perms = [perm for perm in itertools.permutations(self.beacons_positions) if perm[0] == (27, 13)]

         # # Calculate path size for each permutation
        for perm in perms:
            perm_path, perm_path_unk = [], []
            perm_list = list(perm) + [perm[0]]
            for i, beacon in enumerate(perm_list[:-1]):
                perm_path.extend(self.get_path(perm_list[i], perm_list[i + 1], paths[beacon]))
            perm_paths[perm] = perm_path


        for perm in perm_paths.keys():
            # Update shortest path if a shorter path is found
            if len(perm_paths[perm]) < shortest:
                shortest = len(perm_paths[perm])
                shortest_path = np.array(perm_paths[perm])  
                shortest_permutation = perm



        print("Shortest path with known map {} with len {} and permutation {}".format(shortest_path, shortest, shortest_permutation))

        self.shortest_path = shortest_path

        return


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS * 2 - 1) for i in range(CELLROWS * 2 - 1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c + 1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c + 1) // 3 * 2 - 1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c // 3 * 2] = '-'
                        else:
                            None

            i = i + 1


host = "localhost"
pos = 1
mapc = None
rob_name = "pClient1"
filename = "pathing.out"
filename_pathing = "beacon_pathing.out"

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        filename = sys.argv[i + 1]
    elif (sys.argv[i] == "--file-beacon" or sys.argv[i] == "-fb") and i != len(sys.argv) - 1:
        filename_pathing = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host, filename, filename_pathing)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()