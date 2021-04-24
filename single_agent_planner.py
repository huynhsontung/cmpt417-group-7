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

# def push_node(open_list, node):
#     heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

# def pop_node(open_list):
#     _, _, _,curr = heapq.heappop(open_list)
#     return curr


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
        return build_mdd(paths)

    return None  # Failed to find solutions


def find_mdd_path(mdd):
    if not mdd:
        return None

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


# mdds: [mdd1, mdd2, ..., mddn]
# mdd: [[node1], [node2, node3], ... [noden]]
# node: {'loc': (1,2), 'child':[(),()]}
def cardinal_conflict_graph(mdds):
    ccg = {}
    for i in range(len(mdds)):
        mdd1 = mdds[i]
        if i not in ccg:
            ccg[i] = []
        for j in range(len(mdds)):
            if i == j:
                continue
            mdd2 = mdds[j]
            for timestep in range(max(len(mdd1), len(mdd2))):
                # vertex conflict
                if len(get_mdd_nodes(mdd1, timestep)) == len(get_mdd_nodes(mdd2, timestep)) == 1 and get_mdd_nodes(
                        mdd1, timestep)[0]['loc'] == get_mdd_nodes(mdd2, timestep)[0]['loc']:
                    #print("vertex",i," ", j, " ",timestep)
                    ccg[i].append(j)
                    break
                # edge conflict
                if timestep > 0:
                    if len(get_mdd_nodes(mdd1, timestep - 1)) == len(get_mdd_nodes(mdd1, timestep)) == len(get_mdd_nodes(mdd2, timestep - 1)) == len(get_mdd_nodes(mdd2, timestep)) == 1 and\
                            get_mdd_nodes(mdd1, timestep - 1)[0]['loc'] == get_mdd_nodes(mdd2, timestep)[0]['loc'] and get_mdd_nodes(mdd1, timestep)[0]['loc'] == get_mdd_nodes(mdd2, timestep - 1)[0]['loc']:
                        #print("edge",i," ",j," ",timestep)
                        ccg[i].append(j)
                        break

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
    if len(loc) == 1 \
            and len(get_mdd_nodes(mdd1, timestep)) == len(get_mdd_nodes(mdd2, timestep)) == 1 \
            or get_mdd_nodes(mdd1, timestep)[0]['loc'] == get_mdd_nodes(mdd2, timestep)[0]['loc'] == loc:
        return False
    # edge conflict
    if timestep > 0 and len(loc) == 2:
        if len(get_mdd_nodes(mdd1, timestep - 1)) == len(get_mdd_nodes(mdd1, timestep)) == 1 \
                and get_mdd_nodes(mdd1, timestep - 1)[0]['loc'] == loc[0] \
                and get_mdd_nodes(mdd1, timestep)[0]['loc'] == loc[1] \
                or len(get_mdd_nodes(mdd2, timestep - 1)) == len(get_mdd_nodes(mdd2, timestep)) == 1 \
                and get_mdd_nodes(mdd2, timestep)[0]['loc'] == loc[0] \
                and get_mdd_nodes(mdd2, timestep - 1)[0]['loc'] == loc[1]:
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
            ccg, key=lambda v: len(ccg[v]), reverse=True
        )
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
    dg = {}
    for i in range(len(mdds)):
        mdd1 = mdds[i]
        if i not in dg:
            dg[i] = []
        for j in range(len(mdds)):
            if i == j:
                continue
            mdd2 = mdds[j]
            if isDependency(mdd1, mdd2):
                dg[i].append(j)
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


def h_wdg(mdds):
    pass
