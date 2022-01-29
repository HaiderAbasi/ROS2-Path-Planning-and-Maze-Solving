import cv2
import numpy as np

draw_intrstpts = False
debug_mapping = False
display_graph = False
cv2.namedWindow("traversing_crop",cv2.WINDOW_FREERATIO)
class Graph():
    def __init__(self):
        self.graph = {}
        self.start = 0
        self.end = 0

        self.maze = 0


    def add_vertex(self,vertex,neighbor = None,case = None,cost = None):
        print("vertex {} and neighbor {} and case {} ".format(vertex,neighbor,case))
        #if ( (vertex in self.graph.keys()) and (self.graph[vertex][neighbor]!={}) ):
        if ( vertex in self.graph.keys() ):
            #print("What we are appended at loc {} is {} <-> {}".format(vertex,self.graph[vertex],neighbor))
            self.graph[vertex][neighbor] = {}
            self.graph[vertex][neighbor]["case"] = case
            self.graph[vertex][neighbor]["cost"] = cost
            #print("What we appended at loc {} is {}".format(vertex,self.graph[vertex]))
            if display_graph:
                self.displaygraph()
            #cv2.waitKey(0)
        else:
            self.graph[vertex]={}
            self.graph[vertex]["case"]= case
            #self.graph[vertex][neighbor]= {}
            if display_graph:
                self.displaygraph()

    def get_paths(self,start,end,path=[]):
        
        path = path + [start]
        
        if start == end:
            return [path]
        if start not in self.graph.keys():
            return []

        # List to store all possible paths from point A to B
        paths = []

        # Retrieve all connections for that one damn node you are looking it
        for node in self.graph[start].keys():
            # Checking if not already traversed and not a "case" key
            if ( (node not in path) and (node!="case") ):
                new_paths = self.get_paths(node, end,path)
                for p in new_paths:
                    paths.append(p)
        
        return paths
    
    def get_path(self,start,end,path=[]):
        
        path = path + [start]
        
        if start == end:
            return [path]
        if start not in self.graph.keys():
            return []

        # List to store all possible paths from point A to B
        paths = []

        # Retrieve all connections for that one damn node you are looking it
        for node in self.graph[start].keys():
            # No path found yet
            if (paths==[]):
                # Checking if not already traversed and not a "case" key
                if ( (node not in path) and (node!="case") ):
                    new_paths = self.get_path(node, end,path)
                    for p in new_paths:
                        paths.append(p)
        
        return paths

    def get_path_cost(self,start,end,path=[],cost=0,trav_cost=0):
        
        path = path + [start]
        cost = cost + trav_cost

        if start == end:
            return [path],[cost]
        if start not in self.graph.keys():
            return [],0

        # List to store all possible paths from point A to B
        paths = []
        costs = []
        # Retrieve all connections for that one damn node you are looking it
        for node in self.graph[start].keys():
            # No path found yet
            if (paths==[]):
                # Checking if not already traversed and not a "case" key
                if ( (node not in path) and (node!="case") ):

                    new_paths,new_costs = self.get_path_cost(node, end,path,cost,self.graph[start][node]['cost'])
                    for p in new_paths:
                        paths.append(p)
                    for c in new_costs:
                        costs.append(c)
        
        return paths,costs
   
    @staticmethod
    def cords_to_pts(cords):
      return [cord[::-1] for cord in cords]
    
    def draw_shortest_path(self,maze,shortest_path_pts,method="DFS"):
        
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)

        rang = list(range(0,254,25))
        
        depth = maze.shape[0]
        for i in range(len(shortest_path_pts)-1):
            per_depth = (shortest_path_pts[i][1])/depth

            # Blue : []   [0 1 2 3 251 255 251 3 2 1 0] 0-depthperc-0
            # Green :[]     depthperc
            # Red :  [] 100-depthperc
            color = ( 
                      int(255 * (abs(per_depth+(-1*(per_depth>0.5)))*2) ),
                      int(255 * per_depth),
                      int(255 * (1-per_depth))
                    )
            cv2.line(maze_bgr,shortest_path_pts[i] , shortest_path_pts[i+1], color)

        img_str = "maze (Shortest Path) [" +method +"]"
        cv2.namedWindow(img_str,cv2.WINDOW_FREERATIO)
        cv2.imshow(img_str, maze_bgr)
        self.img_shortest_path = maze_bgr

    def get_paths_cost(self,start,end,traversing,path=[],cost=0,trav_cost=0):

        if(start == (146,177)):
            print("testing Starts")

        if path!=[]:
            prev_pt = path[len(path)-1]
            print("Current node = " , start)
            traversing = cv2.line(traversing,(prev_pt[1],prev_pt[0]) , (start[1],start[0]), (0,255,0))
            col = 177
            row = 146
            traversing_crop = traversing[row-40:row+40 , col-40:col+40]
            cv2.imshow("traversing",traversing)
            cv2.imshow("traversing_crop",traversing_crop)
            cv2.waitKey(5)
        path = path + [start]
        cost = cost + trav_cost

        if start == end:
            return [path],[cost]
        if start not in self.graph.keys():
            return [],0


        # List to store all possible paths from point A to B
        paths = []
        costs = []
        # Retrieve all connections for that one damn node you are looking it
        for node in self.graph[start].keys():
            test = self.graph[start]
            # Checking if not already traversed and not a "case" key
            #print("node = ", node)
            if ( (node not in path) and (node!="case") ):

                new_paths,new_costs = self.get_paths_cost(node, end,traversing,path,cost,self.graph[start][node]['cost'])
                traversing = cv2.cvtColor(self.maze, cv2.COLOR_GRAY2BGR)
                if new_paths!= []:
                    #print("Path Found Tonight = ",new_paths)
                    #print("Len(Path Found) = ",len(new_paths[0]))
                    path_pts = self.cords_to_pts(new_paths[0])
                    self.draw_shortest_path(self.maze,path_pts,"chking")
                    #cv2.waitKey(0)
                for p in new_paths:
                    paths.append(p)
                for c in new_costs:
                    costs.append(c)
        
        return paths,costs

  
    def displaygraph(self):
        for key,value in self.graph.items():
            print(" key {} has value {} ".format(key,value))

class maze_converter():
    
    def __init__(self):

        self.Graph = Graph()
        self.graphified = False
        self.maze = 0
        self.mz_crp = 5

        self.img_shortest_path = np.zeros((100,100,3))
        self.shortest_path = 0
        self.shortst_path_found = False

        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_up_right = False

    @staticmethod
    def get_surround_pixel_intensities(maze,curr_row,curr_col):
        #print("[row , col ] = [{} , {}]".format(curr_row,curr_col))
        _,maze = cv2.threshold(maze, 1, 1, cv2.THRESH_BINARY)
        rows = maze.shape[0]
        cols = maze.shape[1]
        top_row = False
        btm_row = False
        lft_col = False
        rgt_col = False


        if (curr_row == 0):
            # Top row => Row above not accesible
            top_row = True
        if (curr_row == (rows-1)):
            # Bottom row ==> Row below not accesible
            btm_row = True
        if (curr_col == 0):
            # Left col ==> Col to the left not accesible
            lft_col = True
        if (curr_col == (cols-1)):
            # Right col ==> Col to the right not accesible
            rgt_col = True

        if( top_row or lft_col ):
            top_left = 0
        else:
            top_left = maze[curr_row-1][curr_col-1]

        if( top_row or rgt_col ):
            top_rgt = 0
        else:
            top_rgt = maze[curr_row-1][curr_col+1]

        if( btm_row or lft_col ):
            btm_left = 0
        else:
            btm_left = maze[curr_row+1][curr_col-1]

        if( btm_row or rgt_col ):
            btm_rgt = 0
        else:
            btm_rgt = maze[curr_row+1][curr_col+1]


        if (top_row):
            top = 0
        else:
            top = maze[curr_row-1][curr_col]

        if (rgt_col):
            rgt = 0
        else:
            rgt = maze[curr_row][curr_col+1]
        
        if (btm_row):
            btm = 0
        else:
            btm = maze[curr_row+1][curr_col]

        if (lft_col):
            lft = 0
        else:
            lft = maze[curr_row][curr_col-1]
        

        #print("  [ top_left , top      , top_rgt  ,lft    , rgt      , btm_left , btm      , btm_rgt   ] [ ",str(top_left)," , ",str(top)," , ",str(top_rgt)," , ",str(lft)," , ",str(rgt)," , ",str(btm_left)," , ",str(btm)," , ",str(btm_rgt)," ] ")

        no_of_pathways = ( top_left + top      + top_rgt  +
                           lft      + 0        + rgt      + 
                           btm_left + btm      + btm_rgt        
                         )
        #print("no_of_pathways [row,col]= [ ",curr_row," , ",curr_col," ] ",no_of_pathways) 

        return top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft,no_of_pathways

    @staticmethod
    def triangle(image,ctr_pt,radius,colour=(0,255,255)):
        # Polygon corner points coordinates
        pts = np.array( [ [ctr_pt[0]        , ctr_pt[1]-radius]  , 
                          [ctr_pt[0]-radius , ctr_pt[1]+radius]  ,
                          [ctr_pt[0]+radius , ctr_pt[1]+radius]   
                        ] 
                        ,np.int32
                      )
        
        pts = pts.reshape((-1, 1, 2))
        
        image = cv2.polylines(image,[pts],True,colour,2)
        return image

    @staticmethod
    def display_connected_nodes(maze_connect,curr_node,that_node,case="Unkown",color=(0,0,255)):
        #maze_connect= cv2.circle(maze_connect, curr_node, 5, (255,0,0))
        #maze_connect= cv2.circle(maze_connect, that_node, 5, (255,0,0))
        print("----------------------) CONNECTED >> {} << ".format(case))
        maze_connect = cv2.line(maze_connect,curr_node,that_node,color,1)
        cv2.imshow("nodes Conected", maze_connect)
        if debug_mapping:
            cv2.waitKey(0)                    
            maze_connect = cv2.line(maze_connect,curr_node,that_node,(255,255,255),1)

    @staticmethod
    def cords_to_pts(cords):
      return [cord[::-1] for cord in cords]

    def draw_shortest_path(self,maze,shortest_path_pts,method="DFS"):
        
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)

        rang = list(range(0,254,25))
        
        depth = maze.shape[0]
        for i in range(len(shortest_path_pts)-1):
            per_depth = (shortest_path_pts[i][1])/depth

            # Blue : []   [0 1 2 3 251 255 251 3 2 1 0] 0-depthperc-0
            # Green :[]     depthperc
            # Red :  [] 100-depthperc
            color = ( 
                      int(255 * (abs(per_depth+(-1*(per_depth>0.5)))*2) ),
                      int(255 * per_depth),
                      int(255 * (1-per_depth))
                    )
            cv2.line(maze_bgr,shortest_path_pts[i] , shortest_path_pts[i+1], color)

        img_str = "maze (Shortest Path) [" +method +"]"
        cv2.namedWindow(img_str,cv2.WINDOW_FREERATIO)
        cv2.imshow(img_str, maze_bgr)
        if method!="DFS":
            self.img_shortest_path = maze_bgr
        


    def connect_neighbors(self,maze,maze_connect,node_row,node_col,step_l,step_up,conn_nodes,case):
        # Check if there is a path surrounding our node
        if (maze[node_row-step_up][node_col-step_l]>0):
            # If there is one --> Check if that path contains a node 
            if ((node_row-step_up,node_col-step_l) in self.Graph.graph.keys()):
                connection_case = self.Graph.graph[(node_row-step_up,node_col-step_l)]["case"]
                conn_nodes += 1
                cost = max(abs(step_l),abs(step_up))
                # If there is a node there --> Connect currentNode <--> That Node and ViceVersa
                self.Graph.add_vertex((node_row,node_col),(node_row-step_up,node_col-step_l),connection_case,cost)
                self.Graph.add_vertex((node_row-step_up,node_col-step_l),(node_row,node_col),case,cost)
                print("\nConnected {} to {} with Case [step_l,step_up] = [ {} , {} ] & Cost -> {}".format((node_row,node_col),(node_row-step_up,node_col-step_l),step_l,step_up,cost))            

                if not self.connected_left:
                    self.display_connected_nodes(maze_connect,(node_col,node_row),(node_col-step_l,node_row-step_up),"LEFT",(0,0,255))
                    self.connected_left = True
                    step_l = 1
                    step_up = 1
                    self.connect_neighbors(maze,maze_connect,node_row,node_col,step_l,step_up,conn_nodes,case)
                elif not self.connected_upleft:
                    self.display_connected_nodes(maze_connect,(node_col,node_row),(node_col-step_l,node_row-step_up),"UPLEFT",(0,128,255))
                    self.connected_upleft = True
                    step_l = 0
                    step_up = 1
                    self.connect_neighbors(maze,maze_connect,node_row,node_col,step_l,step_up,conn_nodes,case)
                elif not self.connected_up:
                    self.display_connected_nodes(maze_connect,(node_col,node_row),(node_col-step_l,node_row-step_up),"UP",(0,255,0))
                    self.connected_up = True
                    step_l = -1
                    step_up = 1                    
                    self.connect_neighbors(maze,maze_connect,node_row,node_col,step_l,step_up,conn_nodes,case)
                elif not self.connected_up_right:
                    self.display_connected_nodes(maze_connect,(node_col,node_row),(node_col-step_l,node_row-step_up),"UPRIGHT",(255,0,0))
                    self.connected_up_right = True

            if not self.connected_up_right:
                if not self.connected_left:
                    step_l+=1
                elif not self.connected_upleft:
                    step_l+=1
                    step_up+=1
                elif not self.connected_up:
                    step_up+=1
                elif not self.connected_up_right:
                    step_l-=1
                    step_up+=1

                self.connect_neighbors(maze,maze_connect,node_row,node_col,step_l,step_up,conn_nodes,case)
        else:
            if not self.connected_left:
                # Basically there is a wall on left so just start looking up lft:)
                self.connected_left = True
                # Looking upleft now
                step_l = 1
                step_up = 1
                self.connect_neighbors(maze,maze_connect,node_row,node_col,step_l,step_up,conn_nodes,case)

            elif not self.connected_upleft:
                # Basically there is a wall up lft so just start looking up :)
                self.connected_upleft = True
                step_l = 0
                step_up = 1
                self.connect_neighbors(maze,maze_connect,node_row,node_col,step_l,step_up,conn_nodes,case)
                

            elif not self.connected_up:
                # Basically there is a wall above so just start looking up-right :)
                self.connected_up = True
                step_l = -1
                step_up = 1
                self.connect_neighbors(maze,maze_connect,node_row,node_col,step_l,step_up,conn_nodes,case)

            elif not self.connected_up_right:
                # Basically there is a wall above so just start looking up-right :)
                self.connected_up_right = True
                step_l = 0
                step_up = 0                
                return


        

    def connect_nodes(self,maze,maze_connect,node_row,node_col,case):
        conn_nodes = 0
        #Start by looking left
        step_l = 1
        step_up = 0
        # Reseting member variables to False initially when looking for nodes to connect
        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_up_right = False
        
        self.connect_neighbors(maze,maze_connect,node_row,node_col,step_l,step_up,conn_nodes,case)




    def one_pass(self,maze):
        # Remove all previously found nodes
        self.Graph.graph.clear()

        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)

        rows = maze.shape[0]
        cols = maze.shape[1]
        
        turns = 0
        junc_3 = 0
        junc_4 = 0
        # Looping over each pixel (car_size_as_a_unit) from left to right ==> bottom to top
        cv2.namedWindow("maze_bgr",cv2.WINDOW_FREERATIO)
        cv2.namedWindow("nodes Conected",cv2.WINDOW_FREERATIO)
        if not debug_mapping:
            maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        else:
            maze_connect = maze.copy()
            
        for row in range(rows):
                print("\n\nAnalyzing Row ( {} )\n\n".format(row))
                for col in range(cols):
                        if debug_mapping:
                            maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)

                        # If we spot a path paint it red :)
                        if (maze[row][col]==255):
                            print("\nMonitoring {}".format((row,col)))

                            top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft, paths = self.get_surround_pixel_intensities(maze.copy(),row,col)

                            if ((row==0) or (row==rows-1) or (col==0) or (col== cols-1)):

                                if (row==0):
                                    # Orange colour
                                    maze_bgr[row][col] = (0,128,255)
                                    cv2.imshow("maze_bgr",maze_bgr)
                                    self.Graph.add_vertex((row,col),case="_Start_")
                                    self.Graph.start = (row,col)
                                    # Bug Found : Top Node Should not look for nodes as their is nothing there
                                    # In case it does look for node and index goes in neg (Wraps around)
                                    # It will loop the whole mat until it can't move anymore
                                    # Interesting consideration -> Entry and Exit should be at one point
                                    #                           -> All other regions should be walled by maze
                                    #                              on exterior
                                    #                           -> Don't look for neighbors for Start Node from itself
                                    #print("\n >>>>>>>>>>>>>>>>> CONNECTING NODES <<<<<<<<<<<<<<<<< \n")
                                    #self.connect_nodes(maze,maze_connect, row,col,case="_Start_")
                                    #print("\n ################# CONNECTED NODES ################## \n")
                                    #self.Graph.displaygraph()
                                    #cv2.waitKey(0)
                                else:
                                    # Green color
                                    maze_bgr[row][col] = (0,255,0)
                                    self.Graph.end = (row,col)
                                    cv2.imshow("maze_bgr",maze_bgr)
                                    self.Graph.add_vertex((row,col),case="_End_")
                                    print("\n >>>>>>>>>>>>>>>>> CONNECTING NODES <<<<<<<<<<<<<<<<< \n")
                                    self.connect_nodes(maze,maze_connect, row,col,case="_End_")
                                    print("\n ################# CONNECTED NODES ################## \n")                                    
                                    #cv2.waitKey(0)                                   
                            # Check if it is a Dead Ebd
                            #elif not ( (top or lft or btm) and (lft or btm or rgt) and(btm or rgt or top) and (rgt or top or lft)     ):
                            elif (paths==1):
                                crop = maze[row-1:row+2,col-1:col+2]
                                #print("Right now we are looking at \n" ,crop)
                                # Green color
                                maze_bgr[row][col] = (0,0,255)
                                if draw_intrstpts:
                                    maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (0,0,255),2)
                                cv2.imshow("maze_bgr",maze_bgr)
                                self.Graph.add_vertex((row,col),case="_DeadEnd_")
                                print("\n >>>>>>>>>>>>>>>>> CONNECTING NODES <<<<<<<<<<<<<<<<< \n")
                                self.connect_nodes(maze,maze_connect, row,col,case="_DeadEnd_")
                                print("\n ################# CONNECTED NODES ################## \n")

                            # Check if it is either a Turn or just an ordinary path
                            #elif not ( (top or lft or btm) and (lft or btm or rgt) and(btm or rgt or top) and (rgt or top or lft)     ):
                            elif (paths==2):
                                crop = maze[row-1:row+2,col-1:col+2]
                                nzero_loc = np.nonzero(crop > 0)
                                nzero_ptA = (nzero_loc[0][0],nzero_loc[1][0])
                                nzero_ptB = (nzero_loc[0][2],nzero_loc[1][2])


                                if not ( ( (2 - nzero_ptA[0])==nzero_ptB[0] ) and 
                                         ( (2 - nzero_ptA[1])==nzero_ptB[1] )     ):
                                    #print("Right now we are looking at [path ==2] \n" ,crop)
                                    # lie on opposite [path]
                                    # Orange color
                                    maze_bgr[row][col] = (255,0,0)
                                    self.Graph.add_vertex((row,col),case="_Turn_")

                                    print("\n >>>>>>>>>>>>>>>>> CONNECTING NODES <<<<<<<<<<<<<<<<< \n")
                                    self.connect_nodes(maze,maze_connect, row,col,case="_Turn_")
                                    print("\n ################# CONNECTED NODES ################## \n")
                                    #if draw_intrstpts:
                                        #maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (255,0,0),2)
                                    turns+=1
                                    cv2.imshow("maze_bgr",maze_bgr)
                            elif (paths>2):
                                if (paths ==3):
                                    crop = maze[row-1:row+2,col-1:col+2]
                                    #print("Right now we are looking at [path ==3] \n" ,crop)
                                    maze_bgr[row][col] = (255,244,128)
                                    self.Graph.add_vertex((row,col),case="_3-Junc_")
                                    print("\n >>>>>>>>>>>>>>>>> CONNECTING NODES <<<<<<<<<<<<<<<<< \n")
                                    self.connect_nodes(maze,maze_connect, row,col,case="_3-Junc_")
                                    print("\n ################# CONNECTED NODES ################## \n")
                                    if draw_intrstpts:
                                        maze_bgr = self.triangle(maze_bgr, (col,row), 10,(144,140,255))

                                    junc_3+=1
                                else:
                                    crop = maze[row-1:row+2,col-1:col+2]
                                    #print("Right now we are looking at [path ==4] \n" ,crop)
                                    maze_bgr[row][col] = (128,0,128)
                                    self.Graph.add_vertex((row,col),case="_4-Junc_")
                                    print("\n >>>>>>>>>>>>>>>>> CONNECTING NODES <<<<<<<<<<<<<<<<< \n")
                                    self.connect_nodes(maze,maze_connect, row,col,case="_4-Junc_")
                                    print("\n ################# CONNECTED NODES ################## \n")
                                    if draw_intrstpts:
                                        cv2.rectangle(maze_bgr,(col-10,row-10) , (col+10,row+10), (255,140,144),2)
                                    junc_4+=1
        
        print("\nInterest Points !!! \n[ Turns , 3_Junc , 4_Junc ] [ ",turns," , ",junc_3," , ",junc_4," ] \n")
        self.Graph.displaygraph()
        if debug_mapping:      
            cv2.waitKey(0)                                   

    def graphify(self,extracted_maze,unit_dim):

        if not self.graphified:
            cv2.imshow("Extracted_Maze [MazeConverter]",extracted_maze)
            extracted_maze_bgr = cv2.cvtColor(extracted_maze, cv2.COLOR_GRAY2BGR)

            _, bw = cv2.threshold(extracted_maze, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)        
            thinned = cv2.ximgproc.thinning(bw)
            #cv2.imshow('thinned Image', thinned)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
            thinned_dilated = cv2.morphologyEx(thinned, cv2.MORPH_DILATE, kernel)
            _, bw2 = cv2.threshold(thinned_dilated, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)        
            thinned = cv2.ximgproc.thinning(bw2)
            cv2.imshow('thinned Image2', thinned)


            thinned_cropped_5pix = thinned[self.mz_crp:thinned.shape[0]-self.mz_crp,
                                           self.mz_crp:thinned.shape[1]-self.mz_crp]
            cv2.imshow('thinned_cropped_5pix', thinned_cropped_5pix)
            
            extracted_maze_cropped_5pix = extracted_maze[self.mz_crp:extracted_maze.shape[0]-self.mz_crp,
                                                         self.mz_crp:extracted_maze.shape[1]-self.mz_crp]
            
            extracted_maze_cropped_5pix = cv2.cvtColor(extracted_maze_cropped_5pix, cv2.COLOR_GRAY2BGR)
            extracted_maze_cropped_5pix[thinned_cropped_5pix>0] = (0,255,255)
            cv2.imshow('extracted_maze_cropped_5pix (Track Overlayed)', extracted_maze_cropped_5pix)
            
            self.one_pass(thinned_cropped_5pix)
            # Graph has been retrieved, Now use it to find paths from start to End
            
            self.graphified = True
            # Saving line_maze to be utilized later
            self.maze = thinned_cropped_5pix

        else:

            if not self.shortst_path_found:

                self.Graph.maze = self.maze

                #paths_N_costs = self.Graph.get_paths_cost(self.Graph.start,self.Graph.end)
                paths_N_costs = self.Graph.get_path_cost(self.Graph.start,self.Graph.end)
                print("paths_N_costs", paths_N_costs)

                paths = paths_N_costs[0]
                costs = paths_N_costs[1]
                min_cost = min(costs)
                shortest_path = paths[costs.index(min_cost)]
                shortest_path_pts = self.cords_to_pts(shortest_path)
                print("shortest_path_pts = {}".format(shortest_path_pts))
                self.draw_shortest_path(self.maze,shortest_path_pts)
                cv2.waitKey(0)

                self.shortest_path = shortest_path_pts
                self.shortst_path_found = True


