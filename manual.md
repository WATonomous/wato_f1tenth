# WATonomous F1Tenth Manual

## What is covered by this doc and prerequisite :

if you are looking at this manual, it is assumed that you got your enviounment setup correctly and everything is working. if you are having touble with it on linux, you can ping Muhtasim Ahsan in micro general. if you are having trouble with windows system, you can ping Rodney Dong in micro general. 

the following will be covered in this doc, but more content may be added in the future : 
1. how to use a gamepad to operate the car in the sim
2. how to map an environment 
3. how to localize in that environment

## Using a Gamepad to control the Car

if you want to to use a gamepad to control the car run, then do the following : 
1. run the following command in the robot container : 
`ros2 launch bringup_robot gamepad.launch.py`

2. you should see the following input than : 
![alt text](/config/images/image5.png)

this means that gamepad input is working. since I am using a ps4 controller I get Sony Computer Entertainment Wireless Controller, but yours may be different depending on what gamepad you are using. Also ensure that when you are using the gamepad, you are in autonomous in the sim.

the following are the controls : 
1. left stick = control steering 
2. right trigger = throtel
3. left trigger = break / reverse

if you find that the steering is too sensitive or not sensitive enough, you can adjust it using the following way : 
`ros2 launch bringup_robot gamepad.launch.py deadzone:=<double value between 0 and 1>`

## Mapping Environments

to map environments do the following : 

1. run `ros2 launch bringup_robot mapping.launch.py`

if it is working than rviz should pop up and you should see somthing like this after a few seconds : 
![Alt Text](/config/images/mapping.gif) 

2. now that we know that mapping is working, you can drive the car around the desired mapping area using your desired teliop method (ensure that you drive very slowly and make a few stops along the path to get the best possible results).

3. to save the map and use it with particle filter, give you map a name and and click save map. this will save the map as a pgm and a yaml (just make sure you move it to your desired location, as the map is saved in the directory where you run the launch comand).


## Localizing Environments

to localize environments do the following : 

1. run `ros2 launch bringup_robot pf_localization.launch.py`. 

if it working you should see something like this : 

![Alt Text](/config/images/localize.gif) 

happy racing