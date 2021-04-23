import time as timer
import heapq
import random

from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def paths_violate_constraint(paths, constraint):
    agent = constraint['agent']
    if 'positive' not in constraint or not constraint['positive']:
        # If not positive constraint then only one path needs to be recalculate
        return [agent]

    agent_list = []
    timestep = constraint['timestep']
    for i in range(len(paths)):
        # if i == agent:
        #     agent_list.append(i)
        #     continue

        path = paths[i]
        loc = constraint['loc'] if i == agent else constraint['loc'][::-1]
        if len(path) <= timestep:
            if i == agent:
                agent_list.append(i)
                continue

            # Check if the path is constrained at goal location
            match = path[-1] == loc[0]
        else:
            # Check if the path is constrained in general
            match = path[timestep] == loc[0] if len(loc) == 1 else path[timestep - 1] == loc[0] and path[timestep] == loc[1]
        
        if (i == agent and not match) or (i != agent and match):
            agent_list.append(i)

        # if len(path) <= timestep:
        #     match = path[-1] == loc[0]
        # else:
        #     match = path[timestep] == loc[0] if len(loc) == 1 else path[timestep - 1] == loc[0] and path[timestep] == loc[1]

        # if match:
        #     agent_list.append(i)

    return agent_list


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    max_time = max(len(path1), len(path2))
    for time in range(max_time):
        v1 = get_location(path1, time)
        v2 = get_location(path2, time)
        if v1 == v2:    # Vertex collision
            return {
                'loc': [v1],
                'timestep': time
            }

        edge1 = [get_location(path1, time - 1), get_location(path1, time)]
        edge2 = [get_location(path2, time - 1), get_location(path2, time)]
        if edge1[0] == edge2[1] and edge1[1] == edge2[0]:   #Edge collision
            return {
                'loc': edge1,
                'timestep': time
            }

    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []
    num_of_agents = len(paths)
    for i in range(num_of_agents):
        for j in range(i + 1, num_of_agents):
            path1 = paths[i]
            path2 = paths[j]
            collision = detect_collision(path1, path2)
            if collision:
                collision['a1'] = i
                collision['a2'] = j
                collisions.append(collision)

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    loc = collision['loc']
    constraint1 = {
        'agent': collision['a1'],
        'loc': loc,
        'timestep': collision['timestep']
    }
    constraint2 = {
        'agent': collision['a2'],
        'loc': loc if len(loc) == 1 else loc[::-1],
        'timestep': collision['timestep']
    }
    return [constraint1, constraint2]


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    rand = random.randint(0,1)
    agent = collision['a1'] if rand == 0 else collision['a2']
    loc = collision['loc'] if rand == 0 else collision['loc'][::-1]
    constraint1 = {
        'agent': agent,
        'loc': loc,
        'timestep': collision['timestep']
    }
    constraint2 = {
        'agent': agent,
        'loc': loc,
        'timestep': collision['timestep'],
        'positive': True
    }
    return [constraint1, constraint2]


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'] + node['cg_heuristic'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path, mdd = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(disjoint_splitting(collision))

        while len(self.open_list) > 0:
            curr = self.pop_node()
            curr_paths = curr['paths']
            if not curr['collisions']:
                self.print_results(curr)
                return curr_paths

            collision = curr['collisions'][0]
            constraints = disjoint_splitting(collision)
            for constraint in constraints:
                node = {
                    'cost': 0,
                    'constraints': curr['constraints'] + [constraint],
                    'paths': list(curr_paths),
                    'collisions': [],
                    'cg_heuristic': compute_cg()
                }
                agents = paths_violate_constraint(node['paths'], constraint)
                has_solution = bool(agents)
                for agent in agents:
                    path, mdd = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, node['constraints'])
                    if path:
                        node['paths'][agent] = path
                    else:
                        has_solution = False
                        break
                    
                if has_solution:
                    node['collisions'] = detect_collisions(node['paths'])
                    node['cost'] = get_sum_of_cost(node['paths'])
                    self.push_node(node)
                    # print(node['paths'])
                    # print(node['collisions'])

        raise BaseException('No solutions')


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print("Solution:        {}".format(node['paths']))
