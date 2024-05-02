import MAPF

from typing import Dict, List, Tuple,Set
from queue import PriorityQueue
import numpy as np

# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

class pyMAPFPlanner:
    def __init__(self, pyenv=None) -> None:
        if pyenv is not None:
            self.env = pyenv.env

        print("pyMAPFPlanner created!  python debug")

    def initialize(self, preprocess_time_limit: int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        pass
        # testlib.test_torch()
        print("planner initialize done... python debug")
        return True
        # raise NotImplementedError()

    def plan(self, time_limit):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (_type_): _description_
        """

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
        if(loc_x >= self.env.rows or loc_y >= self.env.cols or self.env.map[loc] == 1):
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

    def space_time_plan(self,start: int, start_direct: int, end: int, reservation: Set[Tuple[int, int, int]]) -> List[Tuple[int, int]]:
        print(start, start_direct, end)
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
            # print("n=",n)
            _, _, curr = n
        
            curr_location, curr_direction, curr_g, _ = curr

            if (curr_location*4+curr_direction,curr_g) in all_nodes:
                continue
            all_nodes[(curr_location*4+curr_direction,curr_g)]=curr
            if curr_location == end:
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

                if (neighbor_location, -1, curr[2] + 1) in reservation:
                    continue

                if (neighbor_location, curr_location, curr[2] + 1) in reservation:
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

        for v in path:
            print(f"({v[0]},{v[1]}), ", end="")
        print()
        return path

    def sample_priority_planner(self,time_limit:int):
        actions = [MAPF.Action.W] * len(self.env.curr_states)
        reservation = set()  # loc1, loc2, t
        conflicts = set()   # Store conflicts as (agent_id, time_step)

        # Calculate Manhattan distance of each agent from its goal
        manhattan_distances = []
        for i in range(self.env.num_of_agents):
            goal_x, goal_y = self.env.goal_locations[i][0]
            curr_x, curr_y = self.env.curr_states[i].location // self.env.cols, self.env.curr_states[i].location % self.env.cols
            manhattan_distance = abs(goal_x - curr_x) + abs(goal_y - curr_y)
            manhattan_distances.append((i, manhattan_distance))

        # Sort agents based on their Manhattan distances in ascending order
        sorted_agents = [agent_id for agent_id, _ in sorted(manhattan_distances, key=lambda x: x[1])]

        for i in sorted_agents:
            print("start plan for agent", i)
            path = []
            if not self.env.goal_locations[i]:
                print(", which does not have any goal left.")
                path.append((self.env.curr_states[i].location, self.env.curr_states[i].orientation))
                reservation.add((self.env.curr_states[i].location, -1, 1))

        for i in sorted_agents:
            print("start plan for agent", i)
            path = []
            if self.env.goal_locations[i]:
                print("with start and goal:")
                path = self.space_time_plan(
                    self.env.curr_states[i].location,
                    self.env.curr_states[i].orientation,
                    self.env.goal_locations[i][0][0],
                    reservation
                )
            
            if path:
                print("current location:", path[0][0], "current direction:", path[0][1])
                if path[0][0] != self.env.curr_states[i].location:
                    actions[i] = MAPF.Action.FW
                elif path[0][1] != self.env.curr_states[i].orientation:
                    incr = path[0][1] - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[i] = MAPF.Action.CCR

                last_loc = -1
                t = 1
                for p in path:
                    reservation.add((p[0], -1, t))
                    if last_loc != -1:
                        reservation.add((last_loc, p[0], t))
                        if (p[0], t) in reservation:
                            # Conflict detected
                            conflicts.add((i, t))

                    last_loc = p[0]
                    t += 1

                # Ensure that lower-priority agents do not plan conflicting routes
                for agent_id, time_step in conflicts:
                    if manhattan_distances[agent_id][1] > manhattan_distances[i][1]:
                        # Mark actions as wait (W) for lower-priority agents involved in conflicts
                        actions[agent_id] = MAPF.Action.W

        return actions

if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)