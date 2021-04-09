# Robot Path Planning
The robotic path planning problem is a classic. A robot is attempting to navigate its path from the start point to a specified goal region, while avoiding the set of all obstacles.

## IMPORTANT
-> To view Shikha's submission to ERC Inductions, open folder Shikha - ERC and find the required files there. 

-> To view Astitva's submission to ERC Inductions, open folder ERC - Astitva and find the required files there.

## Aim
To implement and test path planning algorithms like RRT(Rapidly exploring Random Trees), PRM(Probabilistic Road Map), Potential Field using python and matplotlib.

## Implementation

### 1. Rapidly exploring Random Trees
In RRT, points are randomly generated within a specified radius and connected to the nearest existing node. Each time a node is created, we check that it lies outside of the obstacles. Furthermore, chaining the node to its closest neighbor must also avoid obstacles. The algorithm ends when a node is generated within the goal region, or a limit is hit.

Check out the Jupyter notebook with RRT implementation [here](https://github.com/fly-zynak/RobotPathPlanning/blob/main/RRT.ipynb). It looks something like this!

![RRT](https://github.com/fly-zynak/RobotPathPlanning/blob/main/Images/RRT.png)

## Team
Astitva Sehgal and Shikha Bhat

