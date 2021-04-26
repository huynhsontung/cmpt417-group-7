import heapq
import time as timer
from pprint import pprint

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


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
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 \
                or child_loc[0] >= len(my_map) \
                or child_loc[1] < 0 \
                or child_loc[1] >= len(my_map[0]):
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
    ##########################
    # Build constraint table whether there is a 'positive' key in constraints
    # or not
    table = dict()
    for constraint in constraints:
        if "positive" not in constraint:
            if constraint['agent'] == agent:
                if (constraint['timestep'], False) not in table:
                    table[(constraint['timestep'], False)] = []
                table[(constraint['timestep'], False)].append(constraint['loc'])
        else:
            if constraint['agent'] != agent and constraint['positive'] == True:
                if (constraint['timestep'], False) not in table:
                    table[(constraint['timestep'], False)] = []
                if len(constraint['loc']) == 1:
                    table[(constraint['timestep'], False)].append(constraint['loc'])
                if len(constraint['loc']) == 2:
                    if (constraint['timestep']-1, False) not in table:
                        table[(constraint['timestep']-1, False)] = []
                    table[(constraint['timestep']-1, False)].append([constraint['loc'][0]])
                    table[(constraint['timestep'], False)].append([constraint['loc'][1]])
                    table[(constraint['timestep'], False)].append([constraint['loc'][1], constraint['loc'][0]])

            if constraint['agent'] == agent and constraint['positive'] == False:
                if (constraint['timestep'], False) not in table:
                    table[(constraint['timestep'], False)] = []
                table[(constraint['timestep'], False)].append(constraint['loc'])

            if constraint['agent'] == agent and constraint['positive'] == True:
                if (constraint['timestep'], True) not in table:
                    table[(constraint['timestep'], True)] = []
                table[(constraint['timestep'], True)].append(constraint['loc'])

    return table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_mdd_nodes(mdd, time):
    if time < 0:
        return mdd[0]
    elif time < len(mdd):
        return mdd[time]
    else:
        return mdd[-1]


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):

    flag1 = False
    flag2 = False
    flag3 = False

    if (next_time, False) in constraint_table:
        flag1 = ([next_loc] in constraint_table[(next_time, False)]) or (
            [curr_loc, next_loc] in constraint_table[(next_time, False)])
    else:
        # check additional constraints
        # the time that agents with higher priorities reached their goal
        # locations
        time = []
        for t, loc in constraint_table.items():
            if [next_loc] in loc and t[0] < 0:
                time.append(t[0])
        flag2 = any(next_time > -t for t in time)

    if (next_time, True) in constraint_table:
        flag3 = ([next_loc] not in constraint_table[(next_time, True)]) and (
            [curr_loc, next_loc] not in constraint_table[(next_time, True)])

    return flag1 or flag2 or flag3


def push_node(open_list, node, path):
    key_tup = (
        node['g_val'] + node['h_val'],
        node['h_val'],
        tuple(path),
        node,
    )
    heapq.heappush(open_list, key_tup)


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def build_mdd(paths):
    path_len = len(paths[0])
    mdd = []

    for ts in range(path_len):
        locs = [p[ts] for p in paths]
        locs_set = set(locs)
        matchings = {
            l: [i for i, x in enumerate(locs) if x == l] for l in locs_set
        }
        mdd_ts = [{
            'loc': l,
            'child': list(set([paths[i][ts + 1] for i in matchings[l]])) if ts != path_len - 1 else []
        } for l in locs_set]

        mdd.append(mdd_ts)

    return mdd

def mdd(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    TIME_LIMIT = 0.1

    paths = []
    optimal_path = None
    optimal_len = -1
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)

    if constraint_table:
        t_occupy_goal = []
        for time, loc in constraint_table.items():
            if [goal_loc] in loc:
                t_occupy_goal.append(time[0])

        if t_occupy_goal:
            earliest_goal_timestep = max(t_occupy_goal) + 1

    max_path_length = 0
    if constraint_table:
        length = max(constraint_table)
        max_path_length += length[0]
    max_path_length += len(my_map) * len(my_map[0])

    root = {
        'loc': start_loc,
        'g_val': 0,
        'h_val': h_value,
        'parent': None,
        'timestep': 0,
    }
    root_path = (start_loc,)
    push_node(open_list, root, root_path)
    closed_list[(root['loc'], 0, root_path)] = root

    start_time = timer.time()
    while len(open_list) > 0:
        curr = pop_node(open_list)
        path = get_path(curr)

        if len(path) > max_path_length:
            break

        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep:
            if optimal_len == -1:
                optimal_path = path
                optimal_len = len(path)

            # Repeat until path is no longer optimal
            if len(path) > optimal_len:
                break

            paths.append(path)
            if timer.time() - start_time > TIME_LIMIT:
                break

            continue

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue

            if my_map[child_loc[0]][child_loc[1]]:  # hits obstacle
                continue

            child = {
                'loc': child_loc,
                'g_val': curr['g_val'] + 1,
                'h_val': h_values[child_loc],
                'parent': curr,
                'timestep': curr['timestep'] + 1,
            }
            if is_constrained(curr['loc'], child['loc'],
                              child['timestep'], constraint_table):
                continue

            child_path = tuple(path + [child_loc])
            prev_locs = (
                curr['parent']['loc'], curr['loc']
            ) if curr['parent'] else (curr['loc'],)
            key = (child['loc'], child['timestep'], prev_locs)
            if key in closed_list:
                existing_node = closed_list[key]
                if compare_nodes(child, existing_node):
                    closed_list[key] = child
                    push_node(open_list, child, child_path)
            else:
                closed_list[key] = child
                push_node(open_list, child, child_path)

    if paths:
        return optimal_path, build_mdd(paths)

    return None, None  # Failed to find solutions


def find_mdd_path(mdd):
    if not mdd:
        return None

    if len(mdd) == 1:
        return [mdd[0][0]['loc']]

    path = []

    for i, mdd_ts in enumerate(mdd):
        if i == 0:
            path.append(mdd_ts[0]['loc'])
            child = mdd_ts[0]['child'][0]
        elif i == len(mdd) - 1:
            path.append(child)
        else:
            node = [x for x in mdd_ts if x['loc'] == child][0]
            path.append(node['loc'])
            child = node['child'][0]

    return path
