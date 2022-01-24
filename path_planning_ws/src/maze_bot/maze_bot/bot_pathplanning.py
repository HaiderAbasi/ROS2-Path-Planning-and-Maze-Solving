from .bot_mapping import maze_converter




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
 
        if self.pos[v] < self.size:
            return True
        return False



class pathfinders():
    def __init__(self):
        pass

    def dijisktra(self,src_vtx):
        # function to find shortest path from src to all vertices

        V = self.vertices
