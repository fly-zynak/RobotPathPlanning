# Robot Path Planning
The robotic path planning problem is a classic. A robot, with certain dimensions, is attempting to navigate between point A and point B while avoiding the set of all obstacles. The robot is able to move through the open area, which is not necessarily discretized.

## Aim:
To implement and test path planning algorithms like RRT(Rapidly exploring Random Trees), PRM(Probabilistic Road Map), Potential Field using python and matplotlib.

### 1. Rapidly exploring Random Trees
In RRT, points are randomly generated and connected to the nearest existing node. Each time a node is created, we check that it lies outside of the obstacles. Furthermore, chaining the node to its closest neighbor must also avoid obstacles. The algorithm ends when a node is generated within the goal region, or a limit is hit.

Check out the Jupyter notebook with RRT implementation [here](https://github.com/fly-zynak/RobotPathPlanning/blob/main/RRT.ipynb). It looks something like this!

![RRT](https://github.com/fly-zynak/RobotPathPlanning/blob/main/Images/RRT.png)

## Team:
Astitva Sehgal and Shikha Bhat

