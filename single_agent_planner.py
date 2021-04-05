import heapq
from enum import Enum


class Direction(Enum):
    UP = (-1, 0)
    DOWN = (1, 0)
    LEFT = (0, -1)
    RIGHT = (0, 1)
    NONE = (0, 0)


def move(loc: tuple[int, int], dir: Direction):
    return loc[0] + dir.value[0], loc[1] + dir.value[1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in Direction:
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    constraint_table = dict()
    # Constraints that always apply in future timesteps
    constraint_table[-1] = []
    for constraint in constraints:
        positive = 'positive' in constraint and constraint['positive']
        if constraint['agent'] != agent and not positive:
            continue

        if constraint['agent'] != agent and positive:
            # Convert positive constraint to negative
            constraint = dict(constraint)
            constraint['positive'] = False
            constraint['agent'] = agent
            constraint['loc'] = constraint['loc'][::-1]

        timestep = constraint['timestep']
        if timestep not in constraint_table:
            constraint_table[timestep] = []

        constraint_table[timestep].append(constraint)

        if 'stop' in constraint and constraint['stop']:
            constraint_table[-1].append(constraint)

    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    constraints = []
    for constraint in constraint_table[-1]:
        if next_time < constraint['timestep']:
            continue

        constraints.append(constraint)

    if next_time in constraint_table:
        constraints += constraint_table[next_time]

    for constraint in constraints:
        positive = 'positive' in constraint and constraint['positive']
        loc_constraint = constraint['loc']
        match = (len(loc_constraint) == 1 and loc_constraint[0] == next_loc) \
            or (len(loc_constraint) == 2 and (loc_constraint[0] == curr_loc and loc_constraint[1] == next_loc))

        # Check vertex and edge constraint
        if match and not positive:
            return True

        # Not satisfying positive constraint
        if not match and positive:
            return True

    return False


def push_node(open_list, node):
    heapq.heappush(
        open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def pop_multiple_nodes(open_list):
    cost, _, _, curr = heapq.heappop(open_list)
    nodes = [curr]
    while len(open_list) > 0:
        next_cost, _, _, next = heapq.heappop(open_list)
        if next_cost == cost:
            nodes.append(next)
        else:
            push_node(open_list, next)
            break

    return nodes


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    env_size = len(my_map) * len(my_map[0])
    open_list = []
    closed_list: dict[tuple[tuple, int], dict] = dict()
    earliest_goal_timestep = 0
    constraint_table = build_constraint_table(constraints, agent)
    last_constraint_time = max(
        list(constraint_table)) if constraint_table else 0
    h_value = h_values[start_loc]
    root = {
        'loc': start_loc,
        'g_val': 0,
        'h_val': h_value,
        'parent': None,
        'time': 0
    }
    push_node(open_list, root)
    closed_list[(root['loc'], 0)] = root

    mdd = []
    path = None
    while len(open_list) > 0:
        nodes = pop_multiple_nodes(open_list)
        mdd_local: dict[tuple[tuple, int], dict] = dict()
        for curr in nodes:
            if curr['time'] > env_size * 5 + last_constraint_time:
                # It's unlikely that there is a valid solution greater than 5
                # times the env size
                break

            if curr['loc'] == goal_loc and curr['time'] >= last_constraint_time:
                path = get_path(curr)
                continue

            for dir in Direction:
                child_loc = move(curr['loc'], dir)
                out_of_bound = child_loc[0] >= len(my_map) \
                    or child_loc[1] >= len(my_map[0]) \
                    or child_loc[0] < 0 \
                    or child_loc[1] < 0
                if out_of_bound or my_map[child_loc[0]][child_loc[1]]:
                    continue

                child_h = h_values[child_loc]
                child = {
                    'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': child_h,
                    'parent': curr,
                    'time': curr['time'] + 1
                }

                if (child_loc, child_h) in mdd_local:
                    mdd_local[(child_loc, child_h)]['prev'].append(curr['loc'])
                else:
                    mdd_local[(child_loc, child_h)] = {
                        'loc': child_loc,
                        'h': child_h,
                        'prev': [curr['loc']]
                    }

                if is_constrained(curr['loc'], child['loc'],
                                  child['time'], constraint_table):
                    continue

                space_time = (child['loc'], child['time'])
                if space_time in closed_list:
                    existing_node = closed_list[space_time]
                    if compare_nodes(child, existing_node):
                        closed_list[space_time] = child
                        push_node(open_list, child)
                else:
                    closed_list[space_time] = child
                    push_node(open_list, child)

        if path:
            return path, mdd

        local_min = min(mdd_local.values(), key=lambda x: x['h'])
        mdd.append([v for v in mdd_local.values() if v['h'] == local_min['h']])

    return None  # Failed to find solutions
