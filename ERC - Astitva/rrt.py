'''
When the test_rrt file is run a path must be found
from the start to the goal(defined in the
test_rrt file) avoiding obstacles present in the environment(obstacle_list)

No changes need to be made in the test_rrt.py file

You're free to add more functions in this file to make 
the code modular.
'''



"""
For the purpose of this implementation, we will be using a goal region, 
i.e. a tiny region around the goal point, that can be used as the goal itself. 
If the path reaches from the start point to this tiny region around the
goal, the path finding is considered to be a Success. 
FOr the purpose of this implementation, where we know that the bounds of 
the environment will be the square of size 10 with one vextrex at origin, 
it is reasonable to choose this tiny region to be a square of size 0.1
around the goal point. 

-> Base assumption: Goal region is small enough such thatn no obstacle lies in goal region
"""



#Your import statements go here

import random
import math
from shapely import geometry
from shapely.geometry import Point,Polygon, LineString
import matplotlib.pyplot as plt


#functions called in test_rrt.py

#the node class for tree data structure
class node():
    
   	
    def __init__(self, name='root', ch=None, par = None):
        self.name = name
        self.ch = []
        if ch is not None:
            for child in ch:
                self.add(child)
        self.par = par

    def __str__(self):
        return self.name

    #function to add child node
    def add(self, node):
        #assert isinstance(node, Tree)
        self.ch.append(node)
        node.par = self
    
    #finds dist between two points    
    def dist(x1,y1,x2,y2):
        d = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        return d
    
    #finds nearest node to point xr,yr in tree (when self = root)    
    def nearest(self,xr,yr,md,pt):
        
        if md >= dist(self.name[0],self.name[1],xr,yr):
                md = dist(self.name[0],self.name[1],xr,yr)
                pt = self
                
        if len(self.ch)==0:
                return (md,pt)
        else:   
            for i in self.ch:
                a =  i.nearest(xr,yr,md,pt)
                if md >= a[0]:
                    md = a[0]
                    pt = a[1]
                else:
                    pass
            return (md,pt)

    #prints the entire tree 
    #in cmd using - to indicate depth/children
    #feel free to try it out   
    def get(self,level):
        if len(self.ch)==0:
            for i in range(level):
                print("-",end="")
            print(str(self.name)+"<-"+str(self.par.name))
            #print(str(self.name))
        else:
            for i in range(level):
                print("-",end="")
            if level !=0:
                print(str(self.name)+"<-"+str(self.par.name))
            else:
                print(str(self.name))
            for i in self.ch :
                i.get(level+1)

    #prints the entire tree in matplotlib
    #feel free to try it out
    def pl(self,level):
        #plt.figure(figsize=(15,10))
        ax = []
        ay = []
        if len(self.ch)==0:
            ax.append(self.par.name[0])
            ax.append(self.name[0])
            ay.append(self.par.name[1])
            ay.append(self.name[1])
            plt.plot(ax,ay,"r.-")
        else:
            if level ==0:
                pass
            else:
                ax.append(self.par.name[0])
                ax.append(self.name[0])
                ay.append(self.par.name[1])
                ay.append(self.name[1])
                plt.plot(ax,ay,"r.-")
            for i in self.ch :
                i.pl(level+1)


    def tb(self,n):
        ax = []
        ay = []
        ax.append(n.name[0])
        ay.append(n.name[1])
        while n.name != self.name:
            n = n.par
            ax.append(n.name[0])
            ay.append(n.name[1])

        return (ax,ay)
        #plt.plot(ax,ay,"b.-")

#generates a random point
def RP(xf,yf):
    x = random.uniform(0,xf)
    y = random.uniform(0,yf)
    return (x,y)

#distance finder
def dist(x1,y1,x2,y2):
    d = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    return d


#the rrt function
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

	c = 0
	xf,yf = 10,10				#bounds
	xs, ys = start[0],start[1]	#start	
	xe,ye = goal[0],goal[1]		#end
	p = []						#polygon array
	num = (len(obstacle_list))
	for i in range(num):
	    p.append(Polygon(obstacle_list[i]))

	root = node((xs,ys)) 		#starting node of tree
	last = root					#last is the last node of the tree till now
	ac = 0
	#------------
	


	#---------

	while c <= 5000:			#max node limit is 5000
	#sampling a random point rp
	    rp = RP(xf,yf)

	#this part of the code samples 1 point in the goal region
	#every 120 moves and checks if it is possible to reach 
	#there in a stright line 
	    don = 0
	    if c%120 == 0:
	        if ac==0:
	            ac = 1
	            rp = (xe,ye)
	            don = 1
	        else:
	            pass
	    else:
	        ac = 0


	#this part of code checks if rp lies within any obstacle or not
	    decider = 0
	    for i in range(num):
	        if p[i].contains(Point(rp)):
	            decider = 1
	            break
	    if decider == 1:
	        continue
	
	#this part of the code finds the nearest node to point rp in existing tree
	    nn = node(rp)						#rp is made a node called nn
	    g = root.nearest(rp[0],rp[1],dist(rp[0],rp[1],root.name[0],root.name[1]),root)

	#this part of code checks if the distance betn. 
	#nearest node and nn is > fixed dist. or not
	#(doesnt check this if we have sample a point 
	# form goal region directly)
	    if don == 0:
	        if g[0]>=0.75:    #for the given sample space, 0.75 is a reasonable fixed distance
	            continue
	    else:
	        print("Checking for goal point directly")	

	#this part checks if sampled point has been previously sampled
	    y = g[1]
	    if nn.name[0] == y.name[0] and y.name[1] == nn.name[1]:
	        continue

	#this pat of code checks if the linkage between the nearest node and nn 
	#will pass through any obstacles or not
	    jl = LineString([nn.name,y.name])
	    decider = 0
	    for i in range(num):
	        if jl.intersects(p[i]):
	            decider = 1
	            break
	    if decider == 1:
	        continue
	
	#This adds the nn node as a child to the nearest node
	    y.add(nn)

	#Prints success if the new sampled node falls
	#in the goal region
	    if nn.name[0] >= xe-0.1 and nn.name[0] <= xe +0.1:
	        if nn.name[1] >= ye-0.1 and nn.name[1] <= ye +0.1:
	            last = nn
	            print("Success!")
	            break

	#this is just a helper print stmt 
	#for me to see if my code is still working
	#or stuck in an infinite loop or
	#worse , lol 

	    if c%300 == 0:
	        print("Current node count: ",c)
	#this is the increment statement,
	#always important guyss
	#unless well, you're using a for loop 
	    c+=1
	
	#----------

	#this part of the code traces back the 
	#final path, and stores the (x_coords,y_coords)
	# of path in pathtup as a tuple


	pathtup = root.tb(last)

	#The code written below is matplotlib code for 
	#plotting the goal region and plotting a boundary
	#This is only done to enhance visualisation
	#the code works fine without this part too.

	plt.figure(1,figsize=(12,9))

	#displays a boundary for the environment
	plt.plot([0-1,xf+1,xf+1,0-1,0-1],[0-1,0-1,yf+1,yf+1,0-1],"y-")
	#displays the goal region 
	plt.plot([xe-0.1,xe+0.1,xe+0.1,xe-0.1,xe-0.1],[ye-0.1,ye-0.1,ye+0.1,ye+0.1,ye-0.1],"b")

	#TO SEE THE ENTIRE TREE GENERATED, UNCOMMENT THE LINE BELOW
	#root.pl(0)
	plt.figure(2,figsize=(12,9))
	#displays a boundary for the environment
	plt.plot([0-1,xf+1,xf+1,0-1,0-1],[0-1,0-1,yf+1,yf+1,0-1],"y-")
	#displays the goal region 
	plt.plot([xe-0.1,xe+0.1,xe+0.1,xe-0.1,xe-0.1],[ye-0.1,ye-0.1,ye+0.1,ye+0.1,ye-0.1],"b")

	#return statement to return pathup
	
	return pathtup

	

	#plt.plot(x,y)
	#plt.figure(figsize=(12,9))
	



def visualize(path,obstacle_list):
	'''
    The matplot code required to visulaize both the path and obstacles in
    the environment go here.

    Args:
        path: Same as used in RRT function.
        obstacle_list (list): list of all obstacles in the envrionment.

    Returns: 
        None
    '''

	plt.figure(1,figsize=(12,9))

	pathx = path[0]
	pathy = path[1]

	p = []						#polygon array
	num = (len(obstacle_list))
	for i in range(num):
	    p.append(Polygon(obstacle_list[i]))

	
	#plotting the obstacles
	for i in range(num):
	    pol = p[i]
	    x,y = pol.exterior.xy
	    plt.plot(x,y,"g.-")

	#plotting the path
	plt.plot(pathx,pathy,"b.-")

	#---------
	#plot the start situation
	plt.figure(2,figsize=(12,9))

	
	for i in range(num):
	    pol = p[i]
	    x,y = pol.exterior.xy
	    plt.plot(x,y,"g.-")

	plt.show()


'''
The results should be visible in the matplot window when test_rrt.py is run
'''
