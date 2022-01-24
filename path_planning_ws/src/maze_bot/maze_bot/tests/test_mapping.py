import cv2

from bot_mapping import maze_converter

maze_converter_ = maze_converter()

def main():
    tiny = cv2.imread("/home/haiderabbasi/Desktop/party.png",cv2.IMREAD_GRAYSCALE)
    print(tiny)
    cv2.namedWindow("tiny_maze",cv2.WINDOW_FREERATIO)
    cv2.imshow("tiny_maze",tiny)
    #cv2.waitKey(0) 
    
    maze_converter_.one_pass(tiny)
    start = (0,4)
    end = (9,6)

    #start = (0,6)
    #end = (9,0)
    paths = maze_converter_.Graph.get_paths_cost(start,end)[0]
    costs = maze_converter_.Graph.get_paths_cost(start,end)[1]
    min_cost = min(costs)
    shortest_path = paths[costs.index(min_cost)]
    shortest_path_pts = maze_converter_.cords_to_pts(shortest_path)

    for i,_ in enumerate(paths):
        #print("\nPath from {} to {} is {}\n".format(start,end,maze_converter_.Graph.get_paths_cost(start,end)))
        print("\nPath from {} to {} is [ {} Cost -> {} ] \n".format(start,end,paths[i],costs[i]))
    
    print("\nShortest Path from {} to {} is [ {} Cost -> {} ] \n".format(start,end,shortest_path,min_cost))

    cv2.waitKey(0)

    
    maze_converter_.draw_shortest_path(tiny,shortest_path_pts)


if __name__ == '__main__':
    main()