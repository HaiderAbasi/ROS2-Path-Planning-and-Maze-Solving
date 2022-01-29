import cv2

from bot_mapping import maze_converter

maze_converter_ = maze_converter()

def main():
    tiny = cv2.imread("/home/haiderabbasi/Desktop/custom.png",cv2.IMREAD_GRAYSCALE)
    #print(tiny)
    cv2.namedWindow("tiny_maze",cv2.WINDOW_FREERATIO)
    cv2.imshow("tiny_maze",tiny)
    #cv2.waitKey(0) 
    
    maze_converter_.one_pass(tiny)
    #start = (0,4)
    start = maze_converter_.Graph.start
    end = maze_converter_.Graph.end
    #end = (9,6)
    #end = (9,4)

    #start = (0,6)
    #end = (9,0)
    
    maze_converter_.Graph.maze = tiny

    traversing = cv2.cvtColor(maze_converter_.Graph.maze, cv2.COLOR_GRAY2BGR)

    paths_N_costs = maze_converter_.Graph.get_paths_cost(start,end,traversing)
    #paths_N_costs = maze_converter_.Graph.get_paths_cost(start,end)
    paths = paths_N_costs[0]
    costs = paths_N_costs[1]
    min_cost = min(costs)
    shortest_path = paths[costs.index(min_cost)]
    shortest_path_pts = maze_converter_.cords_to_pts(shortest_path)
    
    for i,_ in enumerate(paths):
        #print("\nPath from {} to {} is {}\n".format(start,end,maze_converter_.Graph.get_paths_cost(start,end)))
        print("\nPath from {} to {} is [ {} Cost -> {} ] \n".format(start,end,paths[i],costs[i]))
    
    print("\nShortest Path from {} to {} is [ {} Cost -> {} ] \n".format(start,end,shortest_path,min_cost))
    print("No of paths found = ",len(paths))
    cv2.waitKey(0)

    
    maze_converter_.draw_shortest_path(tiny,shortest_path_pts,"Shortest?")
    cv2.waitKey(0)


if __name__ == '__main__':
    main()