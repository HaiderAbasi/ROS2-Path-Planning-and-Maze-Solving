import cv2
import numpy as np

debug_mapping = False

class Graph():
    def __init__(self):
        self.graph = {}


    def add_vertex(self,vertex,neighbor = None,case = None,cost = None):
        print("vertex {} and neighbor {} and case {} ".format(vertex,neighbor,case))
        #if ( (vertex in self.graph.keys()) and (self.graph[vertex][neighbor]!={}) ):
        if ( vertex in self.graph.keys() ):
            #print("What we are appended at loc {} is {} <-> {}".format(vertex,self.graph[vertex],neighbor))
            self.graph[vertex][neighbor] = {}
            self.graph[vertex][neighbor]["case"] = case
            self.graph[vertex][neighbor]["cost"] = cost
            #print("What we appended at loc {} is {}".format(vertex,self.graph[vertex]))
            self.displaygraph()
            #cv2.waitKey(0)
        else:
            self.graph[vertex]={}
            self.graph[vertex]["case"]= case
            #self.graph[vertex][neighbor]= {}
            self.displaygraph()
        
 
    def displaygraph(self):
        for key,value in self.graph.items():
            print(" key {} has value {} ".format(key,value))

class maze_converter():
    
    WALL = 0
    PATH = 255

    def __init__(self):

        self.Graph = Graph()
        self.graphified = False

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
                                    print("\n >>>>>>>>>>>>>>>>> CONNECTING NODES <<<<<<<<<<<<<<<<< \n")
                                    self.connect_nodes(maze,maze_connect, row,col,case="_Start_")
                                    print("\n ################# CONNECTED NODES ################## \n")
                                    #self.Graph.displaygraph()
                                    #cv2.waitKey(0)
                                else:
                                    # Green color
                                    maze_bgr[row][col] = (0,255,0)
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
                                    maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (255,0,0),2)
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
                                    maze_bgr = self.triangle(maze_bgr, (col,row), 20,(144,140,255))
                                    junc_3+=1
                                else:
                                    crop = maze[row-1:row+2,col-1:col+2]
                                    #print("Right now we are looking at [path ==4] \n" ,crop)
                                    maze_bgr[row][col] = (128,0,128)
                                    self.Graph.add_vertex((row,col),case="_4-Junc_")
                                    print("\n >>>>>>>>>>>>>>>>> CONNECTING NODES <<<<<<<<<<<<<<<<< \n")
                                    self.connect_nodes(maze,maze_connect, row,col,case="_4-Junc_")
                                    print("\n ################# CONNECTED NODES ################## \n")
                                    cv2.rectangle(maze_bgr,(col-20,row-20) , (col+20,row+20), (255,140,144),2)
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


            thinned_cropped_5pix = thinned[5:thinned.shape[0]-5,5:thinned.shape[1]-5]
            cv2.imshow('thinned_cropped_5pix', thinned_cropped_5pix)
            extracted_maze_cropped_5pix = extracted_maze[5:extracted_maze.shape[0]-5,5:extracted_maze.shape[1]-5]
            extracted_maze_cropped_5pix = cv2.cvtColor(extracted_maze_cropped_5pix, cv2.COLOR_GRAY2BGR)
            extracted_maze_cropped_5pix[thinned_cropped_5pix>0] = (0,255,255)
            cv2.imshow('extracted_maze_cropped_5pix (Track Overlayed)', extracted_maze_cropped_5pix)

            self.one_pass(thinned_cropped_5pix)
            # Graph has been retrieved, Now use it to find paths from start to End
            self.graphified = True
        else:
            print("Party")

