# Formula Student Driverless Simulation: System overview

FSDS is built around Unreal Engine 4 (the game engine) and the AirSim plugin. 
The game engine does all the graphical rendering, collision simulation and car movement simulation. 
A separate component - the simulator - will handle all controll the simulation, provide external interfaces and store what is happening.

! There will be also something that provides a video feed for livestreaming the simulation. 
  Since it is still unclear how this will work this is not included in this description.

![System overview](images/system-overview.png)