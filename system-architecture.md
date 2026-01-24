# System Architecture
## Goals : 
1. Explain the current arcticture of our current stack
2. Explain how to use this repo for developmnet 

## Current Architecture 
![picture of current stack](/config/current_stack.png "Current F1Tenth Stack")
This diagram ilustrates the current state of the stack we are working towards, our MVP if you will. This MVP stack would allow the car to lap an empty track. however, the car would struggle in real race condition, as it has no means to overtake, and it would simply slow down or stop if another car was on the racing line. Moreover, we would have difficulties localizing in racing conditions, as the other cars can confuse our particle filter.  

## Racing architecture V1
![picture of goal stack](/config/goal_stack.png "Goal F1Tenth Stack")
The goal for the current term is to get to this stack. This would allow us to go racing and possibly overtake other cars. However, this still not perfect as there is still lots of room for us to use more advanced controls, planning and preception algos. 

## How to use the repo ? :

### Why fake odometry ? :
it is unrealistic to run our whole stack all the time. this is becuase part of the stack require nvidia gpu accleration (the particle filter). to remedy this problem, I have made fake odometry node, which provides the nesscary postion, orientaion, angular velocity and velocity data needed for most types of planning and controls algorithm we are currently working on. Along with that, there are some other nodes that are present on the car.

### Working with this repo : 
once you have cloned this repo and have setup your env setup. I suggest **THROUGHLY READING minimumEx.launch.py** in ```/src/robot/bringup_robot/launch``` this provides you with explanations now what nodes **MUST BE ACTIVE** during simulation time. if you do not have these nodes on, you won't be able to properly interface with the sim the way I indented. if you have any question, always feel free to reach our on discord to Muhtasim Ahsan
