import graph_search

def run_ucs(map_path):
    g = graph_search.GridMap(map_path)
    res = graph_search.uniform_cost_search(g.init_pos, g.transition, g.is_goal, graph_search._ACTIONS)
    print("Path:    " + str(res[0]))
    print("Visited: " + str(res[1]))
    g.display_map(res[0],res[1])

if __name__ == '__main__':
    run_ucs('./map0.txt')


