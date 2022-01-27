import cv2
from math import pow ,sqrt
class Heap():
    # --> Heap Class to create heap functionalitites 
    # In this case: Creating prority queue for dijisktra using min-heap
    def __init__(self):
        # Priority queue : a) Maintained in arrays b) Keep track of its size
        #                     c) Keep track of curr pos of each vertex 
        self.array = []
        self.size = 0
        self.posOfVertices = []

    def new_minHeap_node(self,v,dist):
        # Each node will contain two things [Vertex , Distance to that vertex]
        minheap_node = [v,dist]
        return minheap_node

    def swap_nodes(self,a,b):
        # function to swap two nodes [Usage in minHeapify]
        temp = self.array[a]
        self.array[a] = self.array[b]
        self.array[b] = temp

    def minHeapify(self,node_idx):
        # minheapify --> function to convert any part of complete tree to min heap in O(n) time
        smallest = node_idx
        left = node_idx*2+1
        right = node_idx*2+2

        if ( (left<self.size) and (self.array[left][1]<self.array[smallest][1])):
            smallest = left
        if ( (right<self.size) and (self.array[right][1]<self.array[smallest][1])):
            smallest = right

        # if the smallest is not the node_idx, Then we need to do swapping
        if (smallest != node_idx):

            self.posOfVertices[self.array[smallest][0]] = node_idx
            self.posOfVertices[self.array[node_idx][0]] = smallest  

            self.swap_nodes(smallest,node_idx)
            # Recursively call minHeapify until all subnodes part of minheap or no more subnodes left
            self.minHeapify(smallest)

    def extractmin(self):
        # if heap is empty -> return
        if self.size == 0:
            return
        
        # root node 1d Arr : [vertex,cost]
        root = self.array[0]

        # Move lastNode to root of minHeap
        lastNode = self.array[self.size-1]
        self.array[0] = lastNode

        # Update position of each vertex
        self.posOfVertices[root[0]] = self.size-1 
        self.posOfVertices[lastNode[0]] = 0 

        # Decrease size of minHeap by 1
        self.size -= 1
        # Perform min-heapify starting from root
        self.minHeapify(0)

        # Return RootNode which is the minimum element in the minHeap
        return root

    def decreaseKey(self,vertx,dist):
        # Func to modify weight of a specific key to be removed first

        idxofvertex = self.posOfVertices[vertx]

        self.array[idxofvertex][1] = dist

        # Travel up while the complete heap is not heapified

        while ( (idxofvertex>0) and 
                                    (self.array[idxofvertex][1] < self.array[(idxofvertex-1)//2][1]) ):
            
            # Update position of parent and currNode
            self.posOfVertices[self.array[ idxofvertex][0]]        = (idxofvertex-1)//2
            self.posOfVertices[self.array[(idxofvertex-1)//2][0]] =  idxofvertex

            # Swap curr node with parent in self.aray
            self.swap_nodes(idxofvertex, (idxofvertex-1)//2)

            # Navigate to parent index
            idxofvertex = (idxofvertex - 1)//2

    
    # A utility function to check if a given
    # vertex 'v' is in min heap or not
    def isInMinHeap(self, v):
 
        if self.posOfVertices[v] < self.size:
            return True
        return False



class pathfinders():
    def __init__(self):
        self.idxs2vrtxs = {}
        self.shortestpath_found = False
        self.shortest_path = 0
        self.astar_nodes_visited = 0
        self.dijiktra_nodes_visited = 0

    @staticmethod
    def printArr(dist, n):
        print ("Vertex\tDistance from source")
        for i in range(n):
            print ("%d\t\t%d" % (i,dist[i]))
    
    def printShortestVertices(self,parent, n,printorig=False):
        if printorig:
            for i in range(1, n):
                print("{} - {}".format(self.idxs2vrtxs[parent[i]], self.idxs2vrtxs[i]))
        else:
            for i in range(1, n):
                print("% d - % d" % (parent[i], i))

    def ret_shortestroute(self,parent,start,end,route):
        
        route.append(self.idxs2vrtxs[end])

        if (end==start):
            return

        end = parent[end]

        self.ret_shortestroute(parent, start, end, route)

    @staticmethod
    def euc_d(a,b):
        return sqrt( pow( (a[0]-b[0]),2 ) + pow( (a[1]-b[1]),2 ) )

    def dijisktra(self,graph,strt_v,end_v,maze_converter):
        
        # Using list comprehension + enumerate()
        # Key index in Dictionary
        temp = list(graph.graph.items()) 
        strt_v_idx = [idx for idx, key in enumerate(temp) if key[0] == strt_v][0]

        # printing result 
        print("Index of search key is : " + str(strt_v_idx))

        # function to find shortest path from src to all vertices

        V = len(graph.graph.keys())
        print("Total vertices are = ",V)
        print("strt_v is = ",strt_v)

        dist = []
        vrtxs2idxs = {}
        idxs2vrtxs = {}
        # List to store contructed MST
        parent = []

        # Creating an object of heap class to be used as priority queue
        minHeap = Heap()

        for itr,v in enumerate(graph.graph.keys()):
            # Set inf dist for all vertces at initializaiton stage
            dist.append(1e7)
            parent.append(-1)
            minHeap.array.append(minHeap.new_minHeap_node(itr, dist[itr]))
            vrtxs2idxs[v] = itr
            idxs2vrtxs[itr] = v
            minHeap.posOfVertices.append(itr)
        
        self.idxs2vrtxs = idxs2vrtxs

        print("\n\nminHeap.array = ",minHeap.array)
        print("\n\nminHeap.posOFvertices = ",minHeap.posOfVertices)

        # Once minHeap Array and Pos is initialized
        # Set distance of start vertex to 0 to start removing start vertex frst
        minHeap.posOfVertices[strt_v_idx] = strt_v_idx
        dist[strt_v_idx] = 0
        minHeap.decreaseKey(strt_v_idx, dist[strt_v_idx])


        # Size of minHeap at initalization is V (Total number of vertices)
        minHeap.size = V



        # In the following loop min heap contains all nodes
        # whose shortest distance is not yet finalized.
        while (minHeap.size != 0):
            self.dijiktra_nodes_visited +=1
            # Start by extracting the vertex with minDist
            # Which will be our strtVrtx as we set its dist to 0
            curr_top = minHeap.extractmin()
            print("\n\ncurr_top = ",idxs2vrtxs[curr_top[0]])
            #cv2.waitKey(0)
            u = curr_top[0]

            # Traverse through all adjacent vertices of vertex u and update distance values
            for pCrawl in graph.graph[idxs2vrtxs[u]]:
                if pCrawl != "case":
                    print("Vertex adjacent to {} is {}".format(idxs2vrtxs[curr_top[0]],pCrawl))

                    #  Extracting each adjacent vertex
                    v = pCrawl

                    # If we have not found shortest distance to v and distance to v through u is shorter then
                    # origianl then update v known distance
                    if ((minHeap.isInMinHeap(vrtxs2idxs[v]))    and  (dist[u]!=1e7)   and  
                        ( (graph.graph[idxs2vrtxs[u]][pCrawl]["cost"] + dist[u]) < dist[vrtxs2idxs[v]] ) ):
                            
                            dist[vrtxs2idxs[v]] = graph.graph[idxs2vrtxs[u]][pCrawl]["cost"] + dist[u]
                            parent[vrtxs2idxs[v]] = u
                            minHeap.decreaseKey(vrtxs2idxs[v], dist[vrtxs2idxs[v]])
            # End Condition: When our End goal has already been visited. 
            #                This means shortest part to end goal has already been found 
            #     Do   --->              Break Loop
            if (idxs2vrtxs[u]==end_v):
                break

        #self.printArr(dist,V)
        self.printShortestVertices(parent, V)
        
        shortest_path = []

        self.ret_shortestroute(parent,vrtxs2idxs[strt_v],vrtxs2idxs[end_v],shortest_path)
        shortest_path = shortest_path[::-1]

        print("Shortest path from {} to {} is =  {}".format(strt_v,end_v,shortest_path))
        
        shortest_path_pts = maze_converter.cords_to_pts(shortest_path)
        print("shortest_path_pts = {}".format(shortest_path_pts))
        maze_converter.draw_shortest_path(maze_converter.maze,shortest_path_pts,"dijiktra")
        cv2.waitKey(0)
        self.shortest_path = shortest_path_pts
        self.shortestpath_found = True


    def a_star(self,graph,strt_v,end_v,maze_converter):
        
        # Using list comprehension + enumerate()
        # Key index in Dictionary
        temp = list(graph.graph.items()) 
        strt_v_idx = [idx for idx, key in enumerate(temp) if key[0] == strt_v][0]

        # printing result 
        print("Index of search key is : " + str(strt_v_idx))

        # function to find shortest path from src to all vertices

        V = len(graph.graph.keys())
        print("Total vertices are = ",V)
        print("strt_v is = ",strt_v)

        dist = []
        cost2node = {}
        vrtxs2idxs = {}
        idxs2vrtxs = {}
        # List to store contructed MST
        parent = []

        # Creating an object of heap class to be used as priority queue
        minHeap = Heap()

        for itr,v in enumerate(graph.graph.keys()):
            # Set inf dist for all vertces at initializaiton stage
            cost2node[itr] = 0
            dist.append(1e7)
            parent.append(-1)
            minHeap.array.append(minHeap.new_minHeap_node(itr, dist[itr]))
            vrtxs2idxs[v] = itr
            idxs2vrtxs[itr] = v
            minHeap.posOfVertices.append(itr)
        
        self.idxs2vrtxs = idxs2vrtxs

        print("\n\nminHeap.array = ",minHeap.array)
        print("\n\nminHeap.posOFvertices = ",minHeap.posOfVertices)

        # Once minHeap Array and Pos is initialized
        # Set distance of start vertex to 0 to start removing start vertex frst
        minHeap.posOfVertices[strt_v_idx] = strt_v_idx
        # Updating distance of src vertex to [distance to reach src = 0 (Already there) + heuristics[euc_d between src and target]]
        cost2node[strt_v_idx] = 0
        dist[strt_v_idx] = cost2node[strt_v_idx] + self.euc_d(strt_v,end_v)
        minHeap.decreaseKey(strt_v_idx, dist[strt_v_idx])

        # Size of minHeap at initalization is V (Total number of vertices)
        minHeap.size = V

        # In the following loop min heap contains all nodes
        # whose shortest distance is not yet finalized.
        while (minHeap.size != 0):
            self.astar_nodes_visited += 1
            # Start by extracting the vertex with minDist
            # Which will be our strtVrtx as we set its dist to 0
            curr_top = minHeap.extractmin()
            print("\n\ncurr_top = ",idxs2vrtxs[curr_top[0]])
            u = curr_top[0]

            # Traverse through all adjacent vertices of vertex u and update distance values
            for adjNode_n in graph.graph[idxs2vrtxs[u]]:
                if adjNode_n != "case":
                    print("Vertex adjacent to {} is {}".format(idxs2vrtxs[curr_top[0]],adjNode_n))

                    #  Extracting each adjacent vertex
                    v = adjNode_n

                    # If we have not found shortest distance to v and distance to v through u is shorter then
                    # origianl then update v known distance
                    if ((minHeap.isInMinHeap(vrtxs2idxs[v]))    and  (dist[u]!=1e7)   and  
                        ( ( cost2node[u] + graph.graph[idxs2vrtxs[u]][adjNode_n]["cost"] + self.euc_d(adjNode_n,end_v) ) < dist[vrtxs2idxs[v]] ) ):
                            
                            cost2node[vrtxs2idxs[v]] = cost2node[u] + graph.graph[idxs2vrtxs[u]][adjNode_n]["cost"]
                            #Updated AdjNode Dist = cost2node(top) +              cost(adj)                      + euc_dist(adj,end)
                            dist[vrtxs2idxs[v]]   = cost2node[u] + graph.graph[idxs2vrtxs[u]][adjNode_n]["cost"] + self.euc_d(adjNode_n,end_v)
                            parent[vrtxs2idxs[v]] = u
                            minHeap.decreaseKey(vrtxs2idxs[v], dist[vrtxs2idxs[v]])
            # End Condition: When our End goal has already been visited. 
            #                This means shortest part to end goal has already been found 
            #     Do   --->              Break Loop
            if (idxs2vrtxs[u]==end_v):
                break


        #self.printArr(dist,V)
        self.printShortestVertices(parent, V)
        
        shortest_path = []

        self.ret_shortestroute(parent,vrtxs2idxs[strt_v],vrtxs2idxs[end_v],shortest_path)
        shortest_path = shortest_path[::-1]

        print("Shortest path from {} to {} is =  {}".format(strt_v,end_v,shortest_path))
        
        shortest_path_pts = maze_converter.cords_to_pts(shortest_path)
        print("shortest_path_pts = {}".format(shortest_path_pts))
        maze_converter.draw_shortest_path(maze_converter.maze,shortest_path_pts,"A_star")
        cv2.waitKey(0)
        self.shortest_path = shortest_path_pts
        self.shortestpath_found = True


