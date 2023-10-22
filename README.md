
## Package Description  

This repository is a solution for the assignment given in the course [ROS2 For Beginners](https://www.udemy.com/course/ros2-for-beginners/) | Section: 10

**Assignment name**: **`Turtlesim “Catch Them All”`**

---------------------------

## Objective

  * Multiple turtles should  **`spawn`** at random position.  
  * A **`control-turtle`** will try to reach(catch) nearest-turtle, once it reaches the nearest-turtle's present location, nearest-turtle shall be **`removed`**. 


---------------------------

## Requirements

Following are the requirements for the project:
- ROS **`Foxy`** should be installed in the system. 
- **`Turtlesim`** package should be installed. 

to install **`Turtlesim`**, execute the following command:
```console 
sudo apt-get install ros-foxy-turtlesim 
```

### Solution Outline 

- The solution shall be implemented in cpp and python.
- Create a **`ROS 2`** _cpp package_. 
- There will be two nodes. 
- Node 1: 
    * It shall spawn multiple turtles at random location.
- Node 2: 
    * It shall send(publish) appropriate _velocity commands_ for the **`control-turtle`** and will try to reach the nearest-turtle. 
    * Once the control-turtle reaches the nearest-turtle's position, nearest-turtle should be removed.



## Project Status

### Turtle has been spawned at desired position.  
![goal-](./status_data/t1.gif) 

### Multiple turtles are spawning at given position.   
![goal-2](./status_data/t2.gif) 

### Turtles are spawning at random position.  
![goal-3](./status_data/t3.gif) 

### Control turtle is able to traverse to random pose.  
![goal-4](./status_data/t4.gif) 

### Control turtle is traversing to spawned turtle's pose.  
![goal-4](./status_data/t5.gif) 

### Catched turetle is removed. 
![goal-4](./status_data/t6.gif) 

### catch nearby turtle. 
![goal-4](./status_data/t7.gif) 
