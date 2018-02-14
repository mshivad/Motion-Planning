import graph_search

def run_astar(map_path):
    g = graph_search.GridMap(map_path)
    #res = graph_search.a_star_search(g.init_pos, g.transition, g.is_goal, graph_search._ACTIONS_2, g.uninformed_heuristic_euclidean)
    res = graph_search.a_star_search(g.init_pos, g.transition, g.is_goal, graph_search._ACTIONS_2, g.uninformed_heuristic_manhattan)
    print("Path:    " + str(res[0]))
    print("Visited: " + str(res[1]))
    g.display_map(res[0],res[1])

if __name__ == '__main__':
    run_astar('./map2.txt')


