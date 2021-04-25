from single_agent_planner import get_mdd_nodes, get_sum_of_cost


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
    # print(ccg)
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


def weight(mdd1, mdd2, solver):
    start1 = mdd1[0][0]['loc']
    start2 = mdd2[0][0]['loc']
    starts = [start1, start2]

    goal1 = mdd1[-1][0]['loc']
    goal2 = mdd2[-1][0]['loc']
    goals = [goal1, goal2]

    cbs = solver(starts, goals)
    paths = cbs.find_solution(True, 2, False)
    conflict_free_costs = get_sum_of_cost(paths)
    optimal_costs = len(mdd1) + len(mdd2) - 2
    w = conflict_free_costs - optimal_costs
    #print(conflict_free_costs, " ", optimal_costs, " ", w)
    return w


def edge_weighted_dependency_graph(mdds, solver):
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
            w = weight(mdd1, mdd2, solver)
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


def h_wdg(mdds, solver):
    ewdg = edge_weighted_dependency_graph(mdds, solver)
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
