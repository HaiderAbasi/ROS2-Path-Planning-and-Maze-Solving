'''
> Purpose :
Module to perform mapping to convert [(top down) maze view ==> traversable graph.]

> Usage :
You can perform mapping by
1) Importing the class (bot_mapper)
2) Creating its object
3) Accessing the object's function of (graphify). 
E.g ( self.bot_mapper.graphify(self.bot_localizer.maze_og) )


> Inputs:
1) Occupancy Grid from localization stage

> Outputs:
1) self.Graph.graph       => Generated graph from provided maze occupancy grid
2) self.maze              => Image displaying only pathways in maze

Author :
Haider Abbasi

Date :
6/04/22
'''
import cv2
import numpy as np

from . import config

draw_intrstpts = True
debug_mapping = False
# Creating Graph Class to store IP and their connected paths
class Graph():

    def __init__(self):
        # Dictionary to store graph
        self.graph = {}
        # Placeholder for start and end of graph
        self.start = 0
        self.end = 0

    # function to add new vertex to graph
    # if neighbor == None  Just add vertex
    #      Otherwise add connection
    def add_vertex(self,vertex,neighbor= None,case = None, cost = None):
        
        # If neighbor is present ==> Add connection
        if vertex in self.graph.keys():
            self.graph[vertex][neighbor] = {}
            self.graph[vertex][neighbor]["case"] = case
            self.graph[vertex][neighbor]["cost"] = cost
        else:
            # Adding vertex to graph
            self.graph[vertex] = {}
            self.graph[vertex]["case"] = case

    # Function to display complete graph
    def displaygraph(self):
        for key,value in self.graph.items():
            print("key {} has value {} ".format(key,value))

# Bot_Mapper Class for performing Stage 2 (mapping) of robot navigation
class bot_mapper():

    def __init__(self):

        # State Variables
        self.graphified = False

        # Cropping control for removing maze boundary
        self.crp_amt = 5

        # Creating a graph object for storing Maze
        self.Graph = Graph()

        # State variables to define the connection status of each vertex
        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False
        # Maze (Colored) for displaying connection between nodes
        self.maze_connect = []

        self.maze_interestPts = []

        # Maze (One-Pass) Input
        self.maze = 0


    # Display connection between nodes with a colored line
    def display_connected_nodes(self,curr_node,neighbor_node,case="Unkown",color=(0,0,255)):
        curr_pixel = (curr_node[1],curr_node[0])
        neighbor_pixel = (neighbor_node[1],neighbor_node[0])
        #self.maze_connect= cv2.circle(self.maze_connect, curr_pixel, 5, (255,0,0))
        #self.maze_connect= cv2.circle(self.maze_connect, neighbor_pixel, 5, (255,0,0))
        print("----------------------) CONNECTED >> {} << ".format(case))
        self.maze_connect = cv2.line(self.maze_connect,curr_pixel,neighbor_pixel,color,1)
        if config.debug and config.debug_mapping:
            cv2.imshow("Nodes Conected", self.maze_connect)
        if debug_mapping:
            cv2.waitKey(0)                    
            self.maze_connect = cv2.line(self.maze_connect,curr_pixel,neighbor_pixel,(255,255,255),1)

    # Connect curr_node to its neighbors in immediate [left -> top-right] region 
    def connect_neighbors(self,maze,node_row,node_col,case,step_l = 1,step_up = 0,totl_cnncted = 0):
        
        curr_node = (node_row,node_col)

        # Check if there is a path around our node        
        if (maze[node_row-step_up][node_col-step_l]>0):
            # There is a path ==> Look for neighbor node to connect
            neighbor_node = (node_row-step_up,node_col-step_l)
            # If potential_neighbor_node is actually a key in graph                
            if neighbor_node in self.Graph.graph.keys():
                neighbor_case = self.Graph.graph[neighbor_node]["case"]
                cost = max(abs(step_l),abs(step_up))
                totl_cnncted +=1

                self.Graph.add_vertex(curr_node,neighbor_node,neighbor_case,cost)
                self.Graph.add_vertex(neighbor_node,curr_node,case,cost)
                print("\nConnected {} to {} with Case [step_l,step_up] = [ {} , {} ] & Cost -> {}".format(curr_node,neighbor_node,step_l,step_up,cost))

                # Vertex <-Connected-> Neighbor ===) Cycle through Next Possible Routes [topleft,top,top_right]
                if not self.connected_left:
                    self.display_connected_nodes(curr_node, neighbor_node,"LEFT",(0,0,255))
                    # Vertex has connected to its left neighbor.                    
                    self.connected_left = True
                    # Check up-Left route now
                    step_l = 1
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
                if not self.connected_upleft:
                    self.display_connected_nodes(curr_node, neighbor_node,"UPLEFT",(0,128,255))
                    # Vertex has connected to its up-left neighbor.
                    self.connected_upleft = True
                    # Check top route now
                    step_l  = 0
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
                if not self.connected_up:
                    self.display_connected_nodes(curr_node, neighbor_node,"UP",(0,255,0))
                    # Vertex has connected to its up neighbor.
                    self.connected_up = True
                    # Check top-right route now
                    step_l  = -1
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
                if not self.connected_upright:
                    self.display_connected_nodes(curr_node, neighbor_node,"UPRIGHT",(255,0,0))
                    # Vertex has connected to its up-right neighbor.
                    self.connected_upright = True

            # Still searching for node to connect in a respective direction
            if not self.connected_upright:
                if not self.connected_left:
                    # Look a little more left, You'll find it ;)
                    step_l +=1
                elif not self.connected_upleft:
                    # Look a little more (diagnolly) upleft, You'll find it ;)
                    step_l+=1
                    step_up+=1
                elif not self.connected_up:
                    # Look a little more up, You'll find it ;)
                    step_up+=1
                elif not self.connected_upright:
                    # Look a little more upright, You'll find it ;)
                    step_l-=1
                    step_up+=1
                self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
        else:
            # No path in the direction you are looking, Cycle to next direction
            if not self.connected_left:
                # Basically there is a wall on left so just start looking up lft:)
                self.connected_left = True
                # Looking upleft now
                step_l = 1
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case,step_l,step_up,totl_cnncted)
            elif not self.connected_upleft:
                # Basically there is a wall up lft so just start looking up :)
                self.connected_upleft = True
                step_l = 0
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, totl_cnncted)
                

            elif not self.connected_up:
                # Basically there is a wall above so just start looking up-right :)
                self.connected_up = True
                step_l = -1
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, totl_cnncted)

            elif not self.connected_upright:
                # Basically there is a wall above so just start looking up-right :)
                self.connected_upright = True
                step_l = 0
                step_up = 0                
                return     
    
    # function to draw a triangle around a point    
    @staticmethod
    def triangle(image,ctr_pt,radius,colour=(0,255,255),thickness=2):
        # Polygon corner points coordinates
        pts = np.array( [ [ctr_pt[0]        , ctr_pt[1]-radius]  , 
                          [ctr_pt[0]-radius , ctr_pt[1]+radius]  ,
                          [ctr_pt[0]+radius , ctr_pt[1]+radius]   
                        ] 
                        ,np.int32
                      )
        
        pts = pts.reshape((-1, 1, 2))
        
        image = cv2.polylines(image,[pts],True,colour,thickness)
        return image
    
    # function to get surrounding pixels intensities for any point
    @staticmethod
    def get_surround_pixel_intensities(maze,curr_row,curr_col):

        # binary thrsholding and setting (+ values ==> 1 
        #                                 - values ==> 0)
        maze = cv2.threshold(maze, 1, 1, cv2.THRESH_BINARY)[1]

        rows = maze.shape[0]
        cols = maze.shape[1]

        # State variables , If our point is at a boundary condition
        top_row = False
        btm_row = False
        lft_col = False
        rgt_col = False

        # Checking if there is a boundary condition
        if (curr_row==0):
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

        # Extracting surround pixel intensities and Addressing boundary conditions (if present)
        if (top_row or lft_col):
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
        
        # If the point we are at is anywhere on the top row, Then
        #             ===> Its top pixel is definitely not accesible
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

        no_of_pathways = ( top_left + top      + top_rgt  +
                           lft      + 0        + rgt      + 
                           btm_left + btm      + btm_rgt        
                         )
        if no_of_pathways>2:  
            print("  [ top_left , top      , top_rgt  ,lft    , rgt      , btm_left , btm      , btm_rgt   ] \n [ ",str(top_left)," , ",str(top)," , ",str(top_rgt)," ,\n   ",str(lft)," , ","-"," , ",str(rgt)," ,\n   ",str(btm_left)," , ",str(btm)," , ",str(btm_rgt)," ] ")
            print("\nno_of_pathways [row,col]= [ ",curr_row," , ",curr_col," ] ",no_of_pathways) 

        return top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft,no_of_pathways
    # Reset state parameters of each vertex connection
    def reset_connct_paramtrs(self):
        # Reseting member variables to False initially when looking for nodes to connect
        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False

    def one_pass(self,maze):

        # Remove previously found nodes
        self.Graph.graph.clear()

        # Initalizing Maze_connect with Colored Maze
        self.maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        cv2.namedWindow("Nodes Conected",cv2.WINDOW_FREERATIO)

        # Initialize counts of Ip's
        turns = 0
        junc_3 = 0
        junc_4 = 0

        # Converting maze to Colored for Identifying discovered Interest Points
        #maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        maze_bgr = np.zeros((maze.shape[0],maze.shape[1],3),np.uint8)
        # Creating a window to display Detected Interest Points
        cv2.namedWindow("Maze (Interest Points)",cv2.WINDOW_FREERATIO)
        rows = maze.shape[0]
        cols = maze.shape[1]

        # Looping over each pixel from left to right ==> bottom to top
        for row in range(rows):
            for col in range(cols):

                if (maze[row][col]==255):
                    if debug_mapping:
                        # Re-Initalizing Maze_connect with Colored Maze
                        self.maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
                    # Probable IP => Find Surrounding Pixel Intensities
                    top_left,top,top_rgt,rgt,btm_rgt,btm,btm_left,lft, paths = self.get_surround_pixel_intensities(maze.copy(),row,col)

                    if ( (row==0) or (row == (rows-1)) or (col==0) or (col == (cols-1)) ):
                        if (row == 0):
                            # Start
                            maze_bgr[row][col] = (0,128,255)
                            if config.debug and config.debug_mapping:
                                cv2.imshow("Maze (Interest Points)",maze_bgr)
                            # Adding [Found vertex to graph & maze entry to graph-start]
                            self.Graph.add_vertex((row,col),case="_Start_")
                            self.Graph.start = (row,col)

                        else:
                            # End (MAze Exit)
                            maze_bgr[row][col] = (0,255,0)
                            if config.debug and config.debug_mapping:
                                cv2.imshow("Maze (Interest Points)",maze_bgr)
                            # Adding [Found vertex to graph & maze exit to graph-end]
                            self.Graph.add_vertex((row,col),case="_End_")
                            self.Graph.end = (row,col)
                            # Connecting vertex to its neighbor (if-any)
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "_End_")

                    # Check if it is a (Dead End)
                    elif (paths==1):
                        crop = maze[row-1:row+2,col-1:col+2]
                        print(" ** [Dead End] ** \n" ,crop)
                        maze_bgr[row][col] = (0,0,255)# Red color
                        if draw_intrstpts:
                            maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (0,0,255),4)
                        if config.debug and config.debug_mapping:
                            cv2.imshow("Maze (Interest Points)",maze_bgr)
                        # Adding [Found vertex to graph]
                        self.Graph.add_vertex((row,col),case = "_DeadEnd_")
                        # Connecting vertex to its neighbor (if-any)
                        self.reset_connct_paramtrs()
                        self.connect_neighbors(maze, row, col, "_DeadEnd_")

                    # Check if it is either a *Turn* or just an ordinary path
                    elif (paths==2):
                        crop = maze[row-1:row+2,col-1:col+2]
                        nzero_loc = np.nonzero(crop > 0)
                        nzero_ptA = (nzero_loc[0][0],nzero_loc[1][0])
                        nzero_ptB = (nzero_loc[0][2],nzero_loc[1][2])
                        if not ( ( (2 - nzero_ptA[0])==nzero_ptB[0] ) and 
                                    ( (2 - nzero_ptA[1])==nzero_ptB[1] )     ):
                            #maze_bgr[row][col] = (255,0,0)
                            #if draw_intrstpts:
                                #maze_bgr= cv2.circle(maze_bgr, (col,row), 10, (255,0,0),2)
                            if config.debug and config.debug_mapping:
                                cv2.imshow("Maze (Interest Points)",maze_bgr)
                            # Adding [Found vertex to graph]
                            self.Graph.add_vertex((row,col),case = "_Turn_")
                            # Connecting vertex to its neighbor (if-any)
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "_Turn_")
                            turns+=1
                    # Check if it is either a *3-Junc* or a *4-Junc*
                    elif (paths>2):
                        if (paths ==3):
                            maze_bgr[row][col] = (255,244,128)
                            if draw_intrstpts:
                                maze_bgr = self.triangle(maze_bgr, (col,row), 10,(144,140,255),4)
                            if config.debug and config.debug_mapping:
                                cv2.imshow("Maze (Interest Points)",maze_bgr)
                            # Adding [Found vertex to graph]
                            self.Graph.add_vertex((row,col),case = "_3-Junc_")
                            # Connecting vertex to its neighbor (if-any)
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "_3-Junc_")
                            junc_3+=1                                   
                        else:
                            maze_bgr[row][col] = (128,0,128)
                            if draw_intrstpts:
                                cv2.rectangle(maze_bgr,(col-10,row-10) , (col+10,row+10), (255,215,0),4)
                            if config.debug and config.debug_mapping:
                                cv2.imshow("Maze (Interest Points)",maze_bgr)
                            # Adding [Found vertex to graph]
                            self.Graph.add_vertex((row,col),case = "_4-Junc_")
                            # Connecting vertex to its neighbor (if-any)
                            self.reset_connct_paramtrs()
                            self.connect_neighbors(maze, row, col, "_4-Junc_")
                            junc_4+=1
        self.maze_interestPts = maze_bgr
        print("\nInterest Points !!! \n[ Turns , 3_Junc , 4_Junc ] [ ",turns," , ",junc_3," , ",junc_4," ] \n")

    # (Graphify) :           Main function 
    #              [Usage : (Convert) Maze ==> Graph]
    def graphify(self,extracted_maze):

        # Check graph extracted or not from the maze
        if not self.graphified:

            # Step 1: Peforming thinning on maze to reduce area to paths that car could follow.
            thinned = cv2.ximgproc.thinning(extracted_maze)

            # Step 2: Dilate and Perform thining again to minimize unneccesary interest point (i.e:turns)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
            thinned_dilated = cv2.morphologyEx(thinned, cv2.MORPH_DILATE, kernel)
            _, bw2 = cv2.threshold(thinned_dilated, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)        
            thinned = cv2.ximgproc.thinning(bw2)
            
            # Step 3: Crop out Boundary that is not part of maze
            thinned_cropped = thinned[self.crp_amt:thinned.shape[0]-self.crp_amt,
                                      self.crp_amt:thinned.shape[1]-self.crp_amt]

            # Step 4: Overlay found path on Maze Occupency Grid.
            extracted_maze_cropped = extracted_maze[self.crp_amt:extracted_maze.shape[0]-self.crp_amt,
                                                    self.crp_amt:extracted_maze.shape[1]-self.crp_amt]
            extracted_maze_cropped = cv2.cvtColor(extracted_maze_cropped, cv2.COLOR_GRAY2BGR)
            extracted_maze_cropped[thinned_cropped>0] = (0,255,255)
            
            # Step 5: Identify Interest Points in the path to further reduce processing time
            self.one_pass(thinned_cropped)
            #cv2.waitKey(0)
            self.maze = thinned_cropped
            self.graphified = True
            
            if config.debug and config.debug_mapping:
                cv2.imshow("Extracted_Maze [MazeConverter]",extracted_maze)
                cv2.imshow('Maze (thinned)', thinned)
                cv2.imshow('Maze (thinned*2)', thinned)
                cv2.imshow('Maze (thinned*2)(Cropped)', thinned_cropped)
                cv2.imshow('Maze (thinned*2)(Cropped)(Path_Overlayed)', extracted_maze_cropped)
        else:

            if config.debug and config.debug_mapping:
                cv2.imshow("Nodes Conected", self.maze_connect)
                cv2.imshow("Maze (Interest Points)", self.maze_interestPts)
            else:
                try:
                    cv2.destroyWindow("Nodes Conected")
                    cv2.destroyWindow("Maze (Interest Points)")
                    cv2.destroyWindow("Extracted_Maze [MazeConverter]")
                    cv2.destroyWindow('Maze (thinned)')
                    cv2.destroyWindow('Maze (thinned*2)')
                    cv2.destroyWindow('Maze (thinned*2)(Cropped)')
                    cv2.destroyWindow('Maze (thinned*2)(Cropped)(Path_Overlayed)')
                except:
                    pass





