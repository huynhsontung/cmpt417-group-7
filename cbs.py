import time as timer
import heapq
import random
import functools
from heuristics import h_cg, h_dg, h_wdg, is_cardinal_conflict, is_semi_cardinal_conflict, is_non_cardinal_conflict
from single_agent_planner import compute_heuristics, get_location, get_sum_of_cost, mdd, find_mdd_path

from pprint import pprint


def detect_collision(path1, path2):
    """
    Return all the collisions that occurs between two robot paths (or None if there is no collision)
    There are two types of collisions: vertex collision and edge collision.
    A vertex collision occurs if both robots occupy the same location at the same timestep
    An edge collision occurs if the robots swap their location at the same timestep.
    """
    collisions = []
    length = max(len(path1), len(path2))
    for t in range(length):
        location1 = get_location(path1, t)
        location2 = get_location(path2, t)
        # Vertex collision
        if location1 == location2:
            collision = {'loc': [location1], 'timestep': t}
            collisions.append(collision)
        if t:
            pre_loc1 = get_location(path1, t - 1)
            pre_loc2 = get_location(path2, t - 1)
            # Edge collision
            if [pre_loc1, location1] == [location2, pre_loc2]:
                collision = {'loc': [pre_loc1, location1], 'timestep': t}
                collisions.append(collision)
    return collisions
    # return None # no collisions


def detect_collisions(paths):
    """
    Return a list of all collisions between all robot pairs.
    A collision can be represented as dictionary that contains the id of the two robots,
    the vertex or edge causing the collision, and the timestep at which the collision occurred.
    """
    collisions_table = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            collisions = detect_collision(paths[i], paths[j])
            # if collisions:
            for collision in collisions:
                agents_collisions = {'a1': i, 'a2': j}
                agents_collisions.update(collision)
                collisions_table.append(agents_collisions)
    return collisions_table


def get_collision(collisions, mdds):
    semi_cardinal_conflict = None
    non_cardinal_conflict = None
    for collision in collisions:
        if is_cardinal_conflict(collision, mdds):
            return collision
        elif is_semi_cardinal_conflict(collision, mdds):
            semi_cardinal_conflict = collision
        else: #non cardinal conflict
            non_cardinal_conflict = collision
    
    if semi_cardinal_conflict is not None:
        return semi_cardinal_conflict
    else:
        return non_cardinal_conflict


def standard_splitting(collision):
    """
    Return a list of (two) constraints to resolve the given collision
    Vertex collision: the first constraint prevents the first agent to be at the specified location at the
                     specified timestep, and the second constraint prevents the second agent to be at the
                     specified location at the specified timestep.
    Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
                   specified timestep, and the second constraint prevents the second agent to traverse the
                   specified edge at the specified timestep
    """
    constraints = []
    constraint1 = {
        'agent': collision['a1'],
        'loc': collision['loc'],
        'timestep': collision['timestep'],
    }
    constraint2 = {
        'agent': collision['a2'],
        'loc': collision['loc'],
        'timestep': collision['timestep'],
    }
    if len(collision['loc']) == 2:  # Edge collision
        constraint2['loc'] = [collision['loc'][1], collision['loc'][0]]
    constraints.append(constraint1)
    constraints.append(constraint2)
    return constraints


def disjoint_splitting(collision):
    """
    Return a list of (two) constraints to resolve the given collision
    Vertex collision: the first constraint enforces one agent to be at the specified location at the
                     specified timestep, and the second constraint prevents the same agent to be at the
                     same location at the timestep.
    Edge collision: the first constraint enforces one agent to traverse the specified edge at the
                   specified timestep, and the second constraint prevents the same agent to traverse the
                   specified edge at the specified timestep
    Choose the agent randomly
    """

    constraints = []
    agent = collision['a1']
    if random.randint(0, 1) == 1:
        agent = collision['a2']
    constraint1 = {
        'agent': agent,
        'loc': collision['loc'],
        'timestep': collision['timestep'],
        'positive': True,
    }
    constraint2 = {
        'agent': agent,
        'loc': collision['loc'],
        'timestep': collision['timestep'],
        'positive': False,
    }
    constraints.append(constraint1)
    constraints.append(constraint2)
    return constraints


def paths_violate_constraint(constraint, paths):
    """
    Return a list of agent ids of agents that violate a given positive constraint
    """
    ids = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        # vertex constraint
        if [
            get_location(paths[i], constraint['timestep'])
        ] == constraint['loc']:
            ids.append(i)
            continue
        # edge constraint
        if constraint['timestep']:
            if [
                get_location(paths[i], constraint['timestep']),
                get_location(paths[i], constraint['timestep'] - 1)
            ] == constraint['loc']:
                ids.append(i)
                continue

    return ids


def h_val(mdds, h, solver):
    if h == 1:  # icbs - h_cg
        return h_cg(mdds)
    elif h == 2:  # icbs - h_dg
        return h_dg(mdds)
    elif h == 3:  # icbs - h_wdg
        return h_wdg(mdds, solver)
    else:  # icbs
        return 0


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
        tup = (
            node['cost'] + node['h_value'],
            node['h_value'],
            len(node['collisions']),
            self.num_of_generated,
            node
        )
        heapq.heappush(self.open_list, tup)
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, h=0, p=True, stats={}, timeout=300):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        h       - heuristic type
        p       - print debug messages
        stats   - statistics info of the function
        """
        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {
            'cost': 0,
            'constraints': [],
            'paths': [],
            'collisions': [],
            'h_value': 0,
            'mdds': [],
        }

        for i in range(self.num_of_agents):  # Find initial path for each agent
            mddi = mdd(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                       i, [])  # [{'agent': 0, 'loc': [(3,3)], 'timestep': 3}])
            path = find_mdd_path(mddi)
            root['mdds'].append(mddi)
            root['paths'].append(path)

        root['h_value'] = h_val(root['mdds'], h, solver=functools.partial(CBSSolver, self.my_map))
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        ##############################
        # High-Level Search
        while len(self.open_list) > 0:
            parent = self.pop_node()
            if timer.time() - self.start_time > timeout:
                self.write_stats(stats, parent)
                print("Timeout!")
                return parent['paths']

            if len(parent['collisions']) == 0:
                if p:
                    self.print_results(parent)
                    print('Paths:', parent['paths'])
                    self.write_stats(stats, parent)
                return parent['paths']

            collision = get_collision(parent['collisions'], parent['mdds'])

            constraints = disjoint_splitting(collision) \
                if disjoint else standard_splitting(collision)

            for constraint in constraints:
                child = {
                    'cost': 0,
                    'constraints': parent['constraints'] + [constraint],
                    'paths': parent['paths'].copy(),
                    'collisions': [],
                    'h_value': 0,
                    'mdds': parent['mdds'].copy(),
                }

                agents = paths_violate_constraint(constraint, child['paths'])
                if constraint['agent'] not in agents:
                    agents.append(constraint['agent'])

                has_solution = True
                for i in agents:
                    mddi = mdd(
                        self.my_map,
                        self.starts[i],
                        self.goals[i],
                        self.heuristics[i],
                        i,
                        child['constraints'])
                    path = find_mdd_path(mddi)
                    if path:
                        child['paths'][i] = path
                        child['mdds'][i] = mddi
                    else:
                        has_solution = False
                        break

                if has_solution:
                    child['collisions'] = detect_collisions(child['paths'])
                    #ccg = cardinal_conflict_graph(child['mdds'])
                    child['h_value'] = h_val(child['mdds'], h, solver=functools.partial(CBSSolver, self.my_map))
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)

        raise BaseException('No solutions')

    def write_stats(self, stats, node):
        stats['cpu_time'] = timer.time() - self.start_time
        stats['sum_cost'] = get_sum_of_cost(node['paths'])
        stats['num_expanded'] = self.num_of_expanded
        stats['num_generated'] = self.num_of_generated

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
