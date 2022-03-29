# Motion-Control-of-Mobile-Manipulator-Robot
Simulated trajectory planning and simultaneous odometry with feedback control for the KUKA youBot mobile manipulator, a mobile base with four mecanum wheels and a 5R robot arm, using Python, in CoppeliaSIM software

The project includes 3 cases 

How to run the code:

1. Each folder contains TrajectoryGenerator, NextState, FeedbackControl functions.
2. Each folder contains the WrapperScript for each of the designated task.
3. In order to run any case, run the WrapperScript of the case.
4. To test the functions, they can directly be executed as they contain the testing code. 
5. Respective csv file will be generated to test it in CoppeliaSim.

## Best Case Scenario Simulation in CoppeliaSIM
![Best Case Scenario Simulation in CoppeliaSIM](./Gif/Best%20Case%20-%20gif.gif)

## Overshoot Case Scenario Simulation in CoppeliaSIM
![Overshoot Case Scenario Simulation in CoppeliaSIM](./Gif/overshoot%20-%20gif.gif)