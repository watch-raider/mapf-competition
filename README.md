**Design Document**

Algorithm Choice - Priority Based Search (PBS)

Prioritised algorithms plan paths based on a priority ordering of the agents. They are based on the following simple prioritised planning scheme (Erdmann and Lozano-Perez 1987): Each agent is given a unique priority and computes, in priority order, a minimum-cost path from its start vertex to its target vertex that does not collide with the (already planned) paths of all agents with higher priorities.
Prioritised Planning (PP), which plans a shortest path for each agent one after the other in the order from high to low-priority agents that avoids collisions with the (already planned) paths of higher-priority agents
Local orderings assign temporary priorities to some agents in order to resolve collisions on the fly. Such algorithms require agents to follow their assigned paths and, when an impasse is reached, priorities are assigned dynamically to determine who waits (O’Donnell and Lozano-Perez 1989; Azarm and Schmidt 1997).

**Implementation**

I will use A* search for finding the most efficient paths for agents. 
I will then initialise the priority list by prioritising the agents based on Priortised Planning (PP) which plans a shortest path for each agent one after the other in the order from high to low-priority agents that avoids collisions with the (already planned) paths of higher-priority agents
Perform a depth-first search on the high level to dynamically construct a priority ordering and build a priority tree (PT). When it is faced with a collision, PBS greedily chooses which agent should be given a higher priority. If the new path created is shorter than an already created path, the shorter path will take priority and a new path will be generated with constraints for a longer path to avoid the collision.
However, to avoid the same agents always being selected as high priority I will add a probability that the PBS does not greedily choose which agent should be given higher priority. For example, I will set a 80% probability of the algorithm choosing greedily and in the other 20% of cases the algorithm will choose which agent to prioritise randomly (still a chance the algorithm will choose greedily randomly).

I will be using the High-Level Search of PBS algorithm shown below from the ‘Searching with Consistent Prioritization for Multi-Agent Path Finding’ paper by Hang Ma, Daniel Harabor, Peter, J. Stuckey, Jiaoyang Li, University of Southern California, Sven Koenig. 

**Testing**

PBS scales very well, so I will look to measure success across warehouse style maps with 20, 50 and 100 agents over 100 timesteps
I will also test at different thresholds on when the algorithm chooses greedily in order to see which value has the best performance

**Competition details** - https://www.leagueofrobotrunners.org/
