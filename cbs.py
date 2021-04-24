import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, get_location, get_sum_of_cost, mdd, find_mdd_path, get_mdd_nodes

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
        elif is_non_cardinal_conflict(collision, mdds):
            non_cardinal_conflict = collision
        else:
            semi_cardinal_conflict = collision
    if semi_cardinal_conflict is None:
        return non_cardinal_conflict

    return semi_cardinal_conflict


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
    # if random.randint(0, 1) == 1:
    #     agent = collision['a2']
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


def h_val(mdds, h, my_map, solver=None):
    if h == 1:  # icbs - h_cg
        return h_cg(mdds)
    elif h == 2:  # icbs - h_dg
        return h_dg(mdds)
    elif h == 3:  # icbs - h_wdg
        return h_wdg(mdds, my_map, solver)
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

    def find_solution(self, disjoint=True, h=0, p=True, stats={}):
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

            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['h_value'] = h_val(root['mdds'], h, self.my_map, solver=self.find_solution)
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        ##############################
        # High-Level Search
        while len(self.open_list) > 0:
            parent = self.pop_node()

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
                    child['h_value'] = h_val(child['mdds'], h, self.my_map)
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


def cardinal_conflict_graph(mdds):
    """
    mdds: [mdd1, mdd2, ..., mddn]
    mdd: [[node1], [node2, node3], ... [noden]]
    node: {'loc': (1,2), 'child':[(),()]}
    """
    ccg = {agent: [] for agent in range(len(mdds))}
    for i in range(len(mdds)):
        mdd1 = mdds[i]
        for j in range(i + 1, len(mdds)):
            mdd2 = mdds[j]
            for timestep in range(max(len(mdd1), len(mdd2))):
                # vertex conflict
                if len(get_mdd_nodes(mdd1, timestep)) == len(get_mdd_nodes(mdd2, timestep)) == 1 \
                        and get_mdd_nodes(mdd1, timestep)[0]['loc'] == get_mdd_nodes(mdd2, timestep)[0]['loc']:
                    #print("vertex",i," ", j, " ",timestep)
                    ccg[i].append(j)
                    ccg[j].append(i)
                    break
                # edge conflict
                if timestep > 0:
                    if len(get_mdd_nodes(mdd1, timestep - 1)) == len(get_mdd_nodes(mdd1, timestep)) == len(get_mdd_nodes(mdd2, timestep - 1)) == len(get_mdd_nodes(mdd2, timestep)) == 1 \
                            and get_mdd_nodes(mdd1, timestep - 1)[0]['loc'] == get_mdd_nodes(mdd2, timestep)[0]['loc'] \
                            and get_mdd_nodes(mdd1, timestep)[0]['loc'] == get_mdd_nodes(mdd2, timestep - 1)[0]['loc']:
                        #print("edge",i," ",j," ",timestep)
                        ccg[i].append(j)
                        ccg[j].append(i)
                        break
    print(ccg)
    return ccg


def is_cardinal_conflict(collision, mdds):
    a1 = collision['a1']
    a2 = collision['a2']
    timestep = collision['timestep']
    loc = collision['loc']
    mdd1 = mdds[a1]
    mdd2 = mdds[a2]

    # vertex conflict
    if len(loc) == 1 and len(get_mdd_nodes(mdd1, timestep)) == len(get_mdd_nodes(mdd2, timestep)) == 1 and\
            get_mdd_nodes(mdd1, timestep)[0]['loc'] == get_mdd_nodes(mdd2, timestep)[0]['loc'] == loc:
        return True
    # edge conflict
    if timestep > 0 and len(loc) == 2:
        if len(get_mdd_nodes(mdd1, timestep - 1)) == len(get_mdd_nodes(mdd1, timestep)) == len(get_mdd_nodes(mdd2, timestep - 1)) == len(get_mdd_nodes(mdd2, timestep)) == 1 and\
                get_mdd_nodes(mdd1, timestep - 1)[0]['loc'] == get_mdd_nodes(mdd2, timestep)[0]['loc'] == loc[0] and\
                get_mdd_nodes(mdd1, timestep)[0]['loc'] == get_mdd_nodes(mdd2, timestep - 1)[0]['loc'] == loc[1]:
            return True
    return False


def is_non_cardinal_conflict(collision, mdds):

    a1 = collision['a1']
    a2 = collision['a2']
    timestep = collision['timestep']
    loc = collision['loc']
    mdd1 = mdds[a1]
    mdd2 = mdds[a2]

    # vertex conflict
    if len(loc) == 1 and len(get_mdd_nodes(mdd1, timestep)) == len(get_mdd_nodes(mdd2, timestep)
                                                                   ) == 1 or get_mdd_nodes(mdd1, timestep)[0]['loc'] == get_mdd_nodes(mdd2, timestep)[0]['loc'] == loc:
        return False
    # edge conflict
    if timestep > 0 and len(loc) == 2:
        if len(get_mdd_nodes(mdd1, timestep - 1)) == len(get_mdd_nodes(mdd1, timestep)) == 1 and get_mdd_nodes(mdd1, timestep - 1)[0]['loc'] == loc[0] and get_mdd_nodes(mdd1, timestep)[0]['loc'] == loc[1] or\
                len(get_mdd_nodes(mdd2, timestep - 1)) == len(get_mdd_nodes(mdd2, timestep)) == 1 and get_mdd_nodes(mdd2, timestep)[0]['loc'] == loc[0] and get_mdd_nodes(mdd2, timestep - 1)[0]['loc'] == loc[1]:
            return False
    return True


def h_cg(mdds):
    ccg = cardinal_conflict_graph(mdds)
    h = 0
    # find the vertex with maximum edges
    sorted_ccg_vertex = sorted(ccg, key=lambda v: len(ccg[v]), reverse=True)
    vertex = sorted_ccg_vertex[0]

    while len(ccg[vertex]) > 0:
        h += 1
        # label the vertex and remove it from ccg
        del ccg[vertex]
        for v in ccg:
            if vertex in ccg[v]:
                ccg[v].remove(vertex)
        sorted_ccg_vertex = sorted(
            ccg, key=lambda v: len(
                ccg[v]), reverse=True)
        vertex = sorted_ccg_vertex[0]
    # print(h)
    return h


def isDependency(mdd1, mdd2):
    visited = []
    stack = []
    root1 = mdd1[0][0]['loc']
    root2 = mdd2[0][0]['loc']
    # goal1 = mdd1[-1][0]['loc']
    # goal2 = mdd2[-1][0]['loc']

    bigRoot = (root1, root2, 0)
    stack.append(bigRoot)

    while(len(stack)):
        bigCurr = stack.pop()
        curr1 = bigCurr[0]
        curr2 = bigCurr[1]
        t = bigCurr[2]

        # if curr1 == goal1 and curr2 == goal2: #exist a path without conflict
        #     return False

        # print(bigCurr)
        if bigCurr not in visited:
            visited.append(bigCurr)

        if curr1 == curr2:  # vertex conflict
            continue

        for node1 in get_mdd_nodes(mdd1, t):
            if node1['loc'] == curr1:
                children1 = node1['child']
        for node2 in get_mdd_nodes(mdd2, t):
            if node2['loc'] == curr2:
                children2 = node2['child']

        for child1 in children1:
            for child2 in children2:
                if child1 is None and child2 is None:  # exist a path without conflict
                    return False
                if child1 is None:  # append a dummy goal node if length of mdd1 is shorter
                    child1 = curr1
                if child2 is None:  # append a dummy goal node if length of mdd2 is shorter
                    child2 = curr2
                if child1 == curr2 and child2 == curr1:  # edge conflict
                    continue
                bigChild = (child1, child2, t + 1)
                if bigChild not in visited:
                    stack.append(bigChild)
    return True


def dependency_graph(mdds):
    dg = {agent: [] for agent in range(len(mdds))}
    for i in range(len(mdds)):
        mdd1 = mdds[i]
        for j in range(i + 1, len(mdds)):
            mdd2 = mdds[j]
            if isDependency(mdd1, mdd2):
                dg[i].append(j)
                dg[j].append(i)
    # print(dg)
    return dg


def h_dg(mdds):
    dg = dependency_graph(mdds)
    h = 0
    # find the vertex with maximum edges
    sorted_ccg_vertex = sorted(dg, key=lambda v: len(dg[v]), reverse=True)
    vertex = sorted_ccg_vertex[0]

    while len(dg[vertex]) > 0:
        h += 1
        # label the vertex and remove it from ccg
        del dg[vertex]
        for v in dg:
            if vertex in dg[v]:
                dg[v].remove(vertex)
        sorted_ccg_vertex = sorted(dg, key=lambda v: len(dg[v]), reverse=True)
        vertex = sorted_ccg_vertex[0]
    # print(h)
    return h


def weight(mdd1, mdd2, my_map):
    start1 = mdd1[0][0]['loc']
    start2 = mdd2[0][0]['loc']
    starts = [start1, start2]

    goal1 = mdd1[-1][0]['loc']
    goal2 = mdd2[-1][0]['loc']
    goals = [goal1, goal2]

    cbs = CBSSolver(my_map, starts, goals)
    paths = cbs.find_solution(True, 2, False)
    conflict_free_costs = get_sum_of_cost(paths)
    optimal_costs = len(mdd1) + len(mdd2) - 2
    w = conflict_free_costs - optimal_costs
    #print(conflict_free_costs, " ", optimal_costs, " ", w)
    return w


def edge_weighted_dependency_graph(mdds, my_map):
    """
    Example:
    format: {vertex: {neighbour_vertex1: edge_weight, neighbour_vertex2: edge_weight}}
    ewdg = {0: {1:1},
            1: {0:1},
            2: {3:1},
            3: {2:1},
            4: {5:4, 6:1},
            5: {4:4, 6:4},
            6: {4:1, 5:4}}
    """
    ewdg = {agent: dict() for agent in range(len(mdds))}
    for i in range(len(mdds)):
        mdd1 = mdds[i]
        for j in range(i + 1, len(mdds)):
            mdd2 = mdds[j]
            w = weight(mdd1, mdd2, my_map)
            if w > 0:  # a1 and a2 are dependent
                ewdg[i][j] = w
                ewdg[j][i] = w
    # print(ewdg)
    return ewdg


def parse_ewdg(ewdg):
    """
    format: {vertex: [num_of_neighbours, sum_of_edge_weight]}
    parse_ewdg = {0:[1,1],
                  1:[1,1],
                  2:[1,1],
                  3:[1,1],
                  4:[2,5],
                  5:[2,8],
                  6:[2,5]}
    """
    table = {vertex: [] for vertex in range(len(ewdg))}

    for k in table.keys():
        num_of_edges = len(ewdg[k])
        total_weight = 0
        for v in ewdg[k].values():
            total_weight += v
        table[k] = [num_of_edges, total_weight]

    return table


def h_wdg(mdds, my_map):
    ewdg = edge_weighted_dependency_graph(mdds, my_map)
    d = parse_ewdg(ewdg)
    # print(d)
    # ewMVC = [0 for i in range(7)] #Edge-weighted Minimun Vertex Cover(MVC)
    # for debug
    h = 0
    d = {
        k: v for k, v in sorted(
            d.items(),
            key=lambda item: (item[1][0], item[1][1]), reverse=True
        )
    }
    curr = next(iter(d))  # first key in sorted dict

    while d[curr][0] != 0:
        h += 1
        #ewMVC[curr] += 1
        remove_edge = []
        for neighbour in ewdg[curr].keys():
            ewdg[curr][neighbour] -= 1  # decrease the edge weight by 1
            ewdg[neighbour][curr] -= 1

            if ewdg[curr][neighbour] == 0:
                remove_edge.append([curr, neighbour])

        for i in range(len(remove_edge)):
            v1 = remove_edge[i][0]
            v2 = remove_edge[i][1]
            del ewdg[v1][v2]
            del ewdg[v2][v1]

        # re-generate d
        d = parse_ewdg(ewdg)
        # sort d
        d = {
            k: v for k,
            v in sorted(
                d.items(),
                key=lambda item: (
                    item[1][0],
                    item[1][1]),
                reverse=True)}

        # get the vertex with the most (num of edges, total weight)
        curr = next(iter(d))  # first key in sorted dict

    # print(ewMVC)
    return h
