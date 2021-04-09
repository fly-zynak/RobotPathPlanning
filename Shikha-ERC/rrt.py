'''
When the test_rrt file is run a path must be found
from the start to the goal(defined in the
test_rrt file) avoiding obstacles present in the environment(obstacle_list)

No changes need to be made in the test_rrt.py file

You're free to add more functions in this file to make 
the code modular.
'''

#Your import statements go here
import matplotlib.pyplot as plt
import matplotlib.path as mpltPath
import math
import random
from shapely.geometry import Point, MultiPoint
from shapely.geometry import Polygon, MultiPolygon, LineString


#tree structure definition
class Tree():

    def __init__(self, data = Point(0,0), children=None, par=None):
        self.data = data
        self.children = []
        if children is not None:
            for child in children:
                self.add_child(child)
        self.par = par

    def add_child(self, node):
        self.children.append(node)
        node.par = self
                
    def __str__(self, level=0):
        ret = "\t"*level+repr(self.data.x)+" "+repr(self.data.y)+"\n"
        for child in self.children:
            ret += child.__str__(level+1)
        return ret

    def __repr__(self):
        return '<tree node representation>'

    #to trace final path
    def tb(self,n):
        ax = []
        ay = []
        ax.append(n.data.x)
        ay.append(n.data.y)
        while n.data != self.data:
            n = n.par
            ax.append(n.data.x)
            ay.append(n.data.y)
        return ax,ay


#defining obstacles as a set of polygons
def obst(arr):
    ans = list()
    for coord in arr:
        m = Polygon(coord)
        ans.append(m)
    t = MultiPolygon(ans)
    return t

#defining goal region as polygon
def dgoal(coord):
    m = Polygon(coord)
    t = MultiPolygon([m])
    return t

#distance between two points
def distance(pt1,pt2):
    ans = math.sqrt((pt1.x-pt2.x)**2 + (pt1.y-pt2.y)**2)
    return ans

#finding nearest neighbour
def nearestNode(pt,root,mind):
    if distance(root.data,pt)<mind:
        mind = distance(root.data,pt)
    ans = root
    for i in root.children:
        d = nearestNode(pt,i,mind)
        if d[0]<mind:
            mind = d[0]
            ans = d[1]
    return (mind, ans)


#check if point is within polygon
def IsInObstacle(arr,pt):
    for i in arr.geoms:
        if i.contains(pt):
            return True
    return False

#linking new point to existing tree
def chain(node,pt):
    last = Tree(pt)
    node.add_child(last)
    return last



#functions called in test_rrt.py

def RRT(start,goal,obstacle_list):
    '''
    Your RRT code goes here

    The search space will be a rectangular space defined by
    (0,0),(0,10),(10,0),(10,10)

    Args:
        start(tuple):start point coordinates.
        goal (tuple):end point coordinates.
        obstacle_list (list): list of all obstacles in the envrionment.

    Returns:
        path: You are free to decide what data structure used here.
    '''
    goalr = [(goal[0]-0.3,goal[1]-0.3),(goal[0]+0.3,goal[1]-0.3), (goal[0]+0.3,goal[1]+0.3), (goal[0]-0.3,goal[1]+0.3)]
    Qgoal = dgoal(goalr)

    obstacles = obst(obstacle_list)
    
    cnt = 0 
    graph = Tree(Point(start[0],start[1])) #Graph containing edges and vertices, initialized as empty
    
    while cnt<5000:
        
        x  = random.random()*10
        y  = random.random()*10
        p = Point(x,y)
        
        if IsInObstacle(obstacles,p):
            continue
            
        n = nearestNode(p,graph,10**6) #find nearest vertex
        line = LineString([n[1].data,p])

        if n[0]>=1:
            p = line.interpolate(1)
            x = p.x
            y = p.y
            line = LineString([n[1].data,p])
            
        if line.crosses(obstacles):
            continue
            
        last = chain(n[1],p)
        
        if IsInObstacle(Qgoal,p):
            print("Found it!")
            return graph.tb(last),Qgoal

        cnt+=1

    return graph.tb(last),Qgoal



def visualize(path,obstacle_list,Qgoal):

    '''
    The matplot code required to visulaize both the path and obstacles in
    the environment go here.

    Args:
        path: Same as used in RRT function.
        obstacle_list (list): list of all obstacles in the envrionment.

    Returns: 
        None
    '''

    plt.figure()

    plt.plot(path[0],path[1],"b.-")

    m = obst(obstacle_list)
    for i in m:
        x,y = i.exterior.xy
        plt.plot(x,y,"black")

    for i in Qgoal:
        x,y = i.exterior.xy
        plt.plot(x,y,"red")

    plt.show()


'''
The results should be visible in the matplot window when test_rrt.py is run
'''
