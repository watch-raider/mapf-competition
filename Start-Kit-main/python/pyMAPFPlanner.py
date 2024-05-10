import MAPF

from typing import Dict, List, Tuple,Set
from queue import PriorityQueue
import numpy as np
import copy

# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W


class pyMAPFPlanner:

    reservation = dict()  # { (loc1, loc2, t) : agent_id }
    agent_map = dict()
    conflicts = set()

    def __init__(self, pyenv=None) -> None:
        if pyenv is not None:
            self.env = pyenv.env

        self.time = -1
        self.paths = []
        self.agent_map = dict()
        self.vertex_conflict = []
        self.edge_conflict = []

        print("pyMAPFPlanner created!  python debug")

    def initialize(self, preprocess_time_limit: int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        # pass

        #self.env.initialize()
        

        print(self.env.rows)
        print(self.env.cols)
        print(self.env.num_of_agents)

        self.paths = [[] for _ in range(self.env.num_of_agents)]

        # testlib.test_torch()
        print("planner initialize done... python debug")
        return True
        # raise NotImplementedError()

    def init_agent_map(self):
        # print("clearing agent map ...")

        self.agent_map.clear()
        [self.agent_map.update({self.env.curr_states[i].location:i}) for i in range(self.env.num_of_agents)]

        # for i in range(self.env.num_of_agents):
        #     self.agent_map.update({self.env.curr_states[i].location:i})
        return

    def plan(self, time_limit):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (_type_): _description_
        """

        self.time = self.time + 1

        # example of only using single-agent search
        return self.sample_priority_planner(time_limit)
        # print("python binding debug")
        # print("env.rows=",self.env.rows,"env.cols=",self.env.cols,"env.map=",self.env.map)
        # raise NotImplementedError("YOU NEED TO IMPLEMENT THE PYMAPFPLANNER!")

    def getManhattanDistance(self, loc1: int, loc2: int) -> int:
        loc1_x = loc1//self.env.cols
        loc1_y = loc1 % self.env.cols
        loc2_x = loc2//self.env.cols
        loc2_y = loc2 % self.env.cols
        return abs(loc1_x-loc2_x)+abs(loc1_y-loc2_y)

    def validateMove(self, loc: int, loc2: int) -> bool:
        loc_x = loc//self.env.cols
        loc_y = loc % self.env.cols
        if(loc_x < 0 or loc_x >= self.env.rows or loc_y < 0 or loc_y >= self.env.cols or self.env.map[loc] == 1):
            return False
        loc2_x = loc2//self.env.cols
        loc2_y = loc2 % self.env.cols
        if(abs(loc_x-loc2_x)+abs(loc_y-loc2_y) > 1):
            return False
        return True

    def getNeighbors(self, location: int, direction: int):
        neighbors = []
        # forward
        candidates = [location+1, location+self.env.cols,
                      location-1, location-self.env.cols]
        forward = candidates[direction]
        new_direction = direction
        if (forward >= 0 and forward < len(self.env.map) and self.validateMove(forward, location)):
            neighbors.append((forward, new_direction))
        # turn left
        new_direction = direction-1
        if (new_direction == -1):
            new_direction = 3
        neighbors.append((location, new_direction))
        # turn right
        new_direction = direction+1
        if (new_direction == 4):
            new_direction = 0
        neighbors.append((location, new_direction))
        # print("debug!!!!!!!", neighbors)
        return neighbors

    def space_time_plan(self,start: int, start_direct: int, end: int, reservation )-> Tuple[List[Tuple[int, int]] , int]:
        # print(start, start_direct, end)
        max_agent = -1
        conflicting_agents = {} # (agent, number of conflicts)
        path = []
        open_list = PriorityQueue()
        all_nodes = {}  # loc+dict, t
        parent={}
        s = (start, start_direct, 0, self.getManhattanDistance(start, end))
        open_list.put((s[3], id(s), s))
        # all_nodes[(start * 4 + start_direct, 0)] = s
        parent[(start * 4 + start_direct, 0)]=None

        while not open_list.empty():
            n=open_list.get()
            # if start==761: print("n=",n)
            _, _, curr = n
        
            curr_location, curr_direction, curr_g, _ = curr

            if (curr_location*4+curr_direction,curr_g) in all_nodes:
                continue
            all_nodes[(curr_location*4+curr_direction,curr_g)]=curr
            # if curr_location == end: 
            if (curr_location == end and 
                (curr_location,-1,self.time + curr[2] + 1) not in reservation and 
                (curr_location,-1,self.time + curr[2] + 2) not in reservation):
                while True:
                    path.append((curr[0], curr[1]))
                    curr=parent[(curr[0]*4+curr[1],curr[2])]
                    if curr is None:
                        break
                    # curr = curr[5]
                path.pop()
                path.reverse()
                break
            
            neighbors = self.getNeighbors(curr_location, curr_direction)

            for neighbor in neighbors:
                neighbor_location, neighbor_direction = neighbor

                potential_conflicting_agent = reservation.get((neighbor_location, -1, self.time + curr[2] + 1))
                if potential_conflicting_agent is not None:
                    if len(conflicting_agents) > 0 and conflicting_agents.get(potential_conflicting_agent) is not None:
                        conflicting_agents[potential_conflicting_agent] = conflicting_agents[potential_conflicting_agent] + 1
                    else:
                        conflicting_agents[potential_conflicting_agent] = 1
                    continue

                potential_conflicting_agent = reservation.get((neighbor_location, curr_location, self.time + curr[2] + 1))
                if potential_conflicting_agent is not None:
                    if len(conflicting_agents) > 0 and conflicting_agents.get(potential_conflicting_agent) is not None:
                        conflicting_agents[potential_conflicting_agent] = conflicting_agents[potential_conflicting_agent] + 1
                    else:
                        conflicting_agents[potential_conflicting_agent] = 1
                    continue

                neighbor_key = (neighbor_location * 4 +
                                neighbor_direction, curr[2] + 1)

                if neighbor_key in all_nodes:
                    old = all_nodes[neighbor_key]
                    if curr_g + 1 < old[2]:
                        old = (old[0], old[1], curr_g + 1, old[3], old[4])
                else:
                    next_node = (neighbor_location, neighbor_direction, curr_g + 1,
                                self.getManhattanDistance(neighbor_location, end))
        
                    open_list.put(
                        (next_node[3] + next_node[2], id(next_node), next_node))
                
                    parent[(neighbor_location * 4 +
                            neighbor_direction, next_node[2])]=curr

        # for v in path:
        #     print(f"({v[0]},{v[1]}), ", end="")
        # print()
        if len(conflicting_agents) > 0: 
            potential_max_agent = max(conflicting_agents, key=conflicting_agents.get)
            if conflicting_agents[potential_max_agent] > 0.3 * len(path):
                max_agent = potential_max_agent
        return path, max_agent

    def sample_priority_planner(self,time_limit:int):
        actions = [MAPF.Action.W] * len(self.env.curr_states)
        # self.reservation = set()  # loc1, loc2, t

        # ### initial reservation makes no sense, because the agent blocks itself
        # if self.time == 0:
        #     for i in range(self.env.num_of_agents):
        #         # print("starting initital reservation for agent", i)
        #         self.reservation.add((self.env.curr_states[i].location, -1, self.time+1))
        #         self.reservation.add((self.env.curr_states[i].location, -1, self.time+2))
        # ### initial reservation makes no sense, because the agent blocks itself

        self.init_agent_map()

        # Calculate Manhattan distance of each agent from its goal
        manhattan_distances = [(i, self.getManhattanDistance(self.env.curr_states[i].location, self.env.goal_locations[i][0][0])) for i in range(self.env.num_of_agents)]

        # Sort agents based on their Manhattan distances in ascending order
        sorted_agents = [agent_id for agent_id, _ in sorted(manhattan_distances, key=lambda x: x[1])]

        for i in sorted_agents:
            # print("start plan for agent", i)
            path = []
            if not self.env.goal_locations[i]:
                print(", which does not have any goal left.")
                path.append((self.env.curr_states[i].location, self.env.curr_states[i].orientation))
                self.reserve_path(path,i)

        for i in sorted_agents:
            if self.paths[i] == []:
                # print("start plan for agent", i)
                path = []
                if self.env.goal_locations[i]:
                    path , max_agent = self.space_time_plan(
                        self.env.curr_states[i].location,
                        self.env.curr_states[i].orientation,
                        self.env.goal_locations[i][0][0],
                        self.reservation
                    )
                self.paths[i] = path.copy()
                # reserve the newly created path
                self.reserve_path(path,i)
                
                # check if there is another agent with lots of conflicts
                if max_agent >= 0:
                    # print("cancelling ", max_agent, " for ", i)
                    #cancel the path of that agent by first removing the reservations, and then empty the path
                    # remove reservations
                    path = self.paths[max_agent].copy()
                    self.remove_reservations(path)
                    self.paths[max_agent] = []
                    # plan a path again for agent i after removing the previous reservations
                    path = self.paths[i].copy()
                    self.remove_reservations(path)
                    self.paths[i] = []
                    if self.env.goal_locations[i]:
                        path , _ = self.space_time_plan(
                            self.env.curr_states[i].location,
                            self.env.curr_states[i].orientation,
                            self.env.goal_locations[i][0][0],
                            self.reservation
                        )
                    self.paths[i] = path.copy()
                    # reserve the newly created path
                    self.reserve_path(path,i)
                    # if the other agent had a higher priority, then plan a new path for it
                    if max_agent < i:
                        path = []
                        if self.env.goal_locations[max_agent]:
                            # print("with start and goal:")
                            path , _ = self.space_time_plan(
                                self.env.curr_states[max_agent].location,
                                self.env.curr_states[max_agent].orientation,
                                self.env.goal_locations[max_agent][0][0],
                                self.reservation
                            )
                        self.paths[max_agent] = path.copy()
                        # reserve the newly created path
                        self.reserve_path(path,max_agent)

        self.vertex_conflict = [0] * len(self.env.map)
        self.edge_conflict = [-1] * len(self.env.map)
        found = self.find_conflicts()
        while found:
            for i in self.conflicts:
                path = self.paths[i].copy()
                # remove reservations
                self.remove_reservations(path)
                next_orientation = self.env.curr_states[i].orientation + 1
                if next_orientation > 3: next_orientation = 0
                path = []
                path.append((self.env.curr_states[i].location, next_orientation))
                self.paths[i] = path.copy()
                self.reserve_path(path,i)
            found = self.find_conflicts()

        for i in range(self.env.num_of_agents):
            path = self.paths[i]
            if path:
                # print("current location:", path[0][0], "current direction:", path[0][1])
                if path[0][0] != self.env.curr_states[i].location:
                    actions[i] = MAPF.Action.FW
                elif path[0][1] != self.env.curr_states[i].orientation:
                    incr = path[0][1] - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[i] = MAPF.Action.CCR
                # if i == 13: print("agent 13 action: ", actions[i])

            if len(self.paths[i]) > 0: 
                self.paths[i].pop(0)

        return actions

    def find_conflicts(self) -> bool:
        self.conflicts.clear()
        found = False
        surrounding_agents = list()
        # for i in range(self.num_of_agents) if self.time % 2 == 1 else reversed(range(self.num_of_agents)):
        for i in range(self.env.num_of_agents):
            if (self.paths[i] != [] and self.paths[i][0][0] != self.env.curr_states[i].location): # this robot moves
                location = self.env.curr_states[i].location
                candidates = [location+1, location+self.env.cols,
                              location-1, location-self.env.cols]
                forward = candidates[self.env.curr_states[i].orientation]
                surrounding_agents.clear()
                if self.agent_map.get(forward) != None: surrounding_agents.append(self.agent_map.get(forward)) # agent in the target cell
                if self.agent_map.get(forward+1) != None: surrounding_agents.append(self.agent_map.get(forward+1)) # agents in the surrounding cells
                if self.agent_map.get(forward+self.env.cols) != None: surrounding_agents.append(self.agent_map.get(forward+self.env.cols)) # agents in the surrounding cells
                if self.agent_map.get(forward-1) != None: surrounding_agents.append(self.agent_map.get(forward-1)) # agents in the surrounding cells
                if self.agent_map.get(forward-self.env.cols) != None: surrounding_agents.append(self.agent_map.get(forward-self.env.cols)) # agents in the surrounding cells
                for agent_j in surrounding_agents:
                    if agent_j == i:
                        continue
                    if ((self.env.curr_states[agent_j].location == forward and  # there is a robot and
                        ((self.paths[agent_j] == [] or self.paths[agent_j][0][0] == forward) or # that robot does not move
                         (self.paths[agent_j] != [] and self.paths[agent_j][0][0] == location)) ) or # or moves in the opposite direction
                        (self.paths[agent_j] != [] and self.paths[agent_j][0][0] == forward)  # or that robot moves to the same place
                    ):
                        if (self.paths[agent_j] != [] and self.paths[agent_j][0][0] == location):   # it is an edge conflict
                            self.edge_conflict[location] = forward
                        else:              # if not edge conflict then it is a vertex conflict
                            self.vertex_conflict[forward] = self.vertex_conflict[forward] +1
                        self.conflicts.add(i)
                        found = True
            # if found: break
        return found

    def reserve_path(self, path:List[Tuple[int, int]], agent:int):
        last_loc = -1
        t = 1
        for p in path:
            self.reservation[(p[0], -1, self.time + t)] = agent
            if last_loc != -1:
                self.reservation[(last_loc, p[0], self.time + t)] = agent
            last_loc = p[0]
            t += 1

    def remove_reservations(self, path:List[Tuple[int, int]]):
        last_loc = -1
        t = 1
        for p in path:
            self.reservation.pop((p[0], -1, self.time + t))
            if last_loc != -1:
                self.reservation.pop((last_loc, p[0], self.time + t))
            last_loc = p[0]
            t += 1


if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)
