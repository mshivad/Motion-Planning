import graph_search
import time

def run_bfs(map_path):
    g = graph_search.GridMap(map_path)
    res = graph_search.bfs(g.init_pos, g.transition, g.is_goal, graph_search._ACTIONS)
    #res = graph_search.bfs(g.init_pos, g.transition, g.is_goal, graph_search._ACTIONS_2)
    #actionset = res[0];
    print("Plan:    " + str(res[0]))
    print("Visited: " + str(res[1]))
    #print res;
    #g.display_map(res[0],res[1])
    y = len(res[0])
    res1 = g.init_pos
    list = []
    for x in range (0, y):
        i=0
        actionset = res[0][i]
        i=+1
        res1=g.bfs_simulator(res1, res[0][x] , graph_search._ACTIONS, action_probability=0.8, neighbour_probability=0.1, ortho_probability=0, simulate=False)
        #res1 = g.bfs_simulator(res1, res[0][x], graph_search._ACTIONS_2, action_probability=0.8, neighbour_probability=0.1, ortho_probability=0.05, simulate=False)
        #res1= graph_search.GridMap.bfs_simulator()
        list.append(res1)
        #print res1;
    print(list)
    g.display_map(list,res[1])

def run_value_iteration(map_path):
    start = time.time()
    g = graph_search.GridMap(map_path)
    actions_list = graph_search._ACTIONS
    #actions_list1 = graph_search._ACTIONS_2
    values = g.findValues(10,-1)
    result=g.value_iteration(0.8,values,actions_list,0.8,0.1,0)
    #result = g.value_iteration(0.8, values, actions_list1, 0.8, 0.1, 0.0)
    print("run time: " + str(time.time() - start))
    print(result[0])
    print(result[1])
    g.display_values(result[0])
    g.display_policy(result[0], result[1])

def run_PI(map_path):
    start = time.time()
    g = graph_search.GridMap(map_path)
    actions_list = graph_search._ACTIONS
    #actions_list1 = graph_search._ACTIONS_2
    values = g.findValues(10, 0)
    policy = g.find_policy(actionset= actions_list, randomize= True)
    #policy = g.find_policy(actionset= actions_list1, randomize= True)
    result = g.PI(0.8,values,actions_list,0.8,0.1,0.0,policy)
    #result = g.PI(0.8,values,actions_list1,0.8,0.1,0,policy)
    print("run time: " +str(time.time() - start))
    print(result[0])
    print(result[1])
    g.display_values(result[0])
    g.display_policy(result[0], result[1])

if __name__ == '__main__':
    #run_value_iteration('./map0.txt')
    run_PI('./map1.txt')
    #run_bfs('./map0.txt')

