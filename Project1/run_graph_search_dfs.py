import graph_search

def run_dfs(map_path):
    g = graph_search.GridMap(map_path)
    res = graph_search.dfs(g.init_pos, g.transition, g.is_goal, graph_search._ACTIONS)
    print("Path:    " + str(res[0]))
    print("Visited: " + str(res[1]))
    g.display_map(res[0],res[1])

if __name__ == '__main__':
    run_dfs('./map1.txt')
