import graph_search

def run_iddfs(map_path):
    g = graph_search.GridMap(map_path)
    res = graph_search.iddfs(g.init_pos, g.transition, g.is_goal, graph_search._ACTIONS, 5)
    print("Path:    " + str(res[0]))
    print("Visited: " + str(res[1]))
    g.display_map(res[0],res[1])

if __name__ == '__main__':
    run_iddfs('./map0.txt')


