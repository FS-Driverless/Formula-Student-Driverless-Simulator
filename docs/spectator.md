# Spectator
The spectator provides an eye into the virtual world from an external computer.
Through a network connection the virtual world state is replicated and shown on the screen.
The user can operate the camera with its mouse and keyboard.

## Launching the spectator

A simulator launched as 'server' will accept external viewers (spectators) to join the game.
The spectators are not able to interact with the world.
Spectators can enter the ip of the server in the main menu to connect to a running simulator.
Multiple simultaneous spectators in a single world should work as well.

> If a spectator loses connection to the server it will go back to the main menu.

If you want to give access to clients from external computers you must ensure the specified port is accessible. 
In most cases you will have to add a firewall rule to allow the traffic.
When behind a router you might need to do some port forwarding.
Both TCP and UDP traffic must be able to travel from client to server on port 7777.

To skip the menu and launch the game as server or spectator directly, you can use the following commandline options:

**Run the game as a server**
```
FSDS.exe /Game/TrainingMap?listen -log
```
This runs the simulator like normal but now external clients (spectators) are welcome to connect.


**Running the spectator**
```
FSDS.exe 0.0.0.0 -log
```
Where `0.0.0.0` is replaced by the external ip of the server.


## Using the spectator

When starting the spectator it will be launched in follow-car mode.
In this mode the camera will always point at the vehicle.
When the vehicle crosses a triggerline, the camera will move to another viewpoint.

You can toggle the follow-car mode using the `F` key on your keyboard.
When not following the car you can use the `wsdaqe` keys to move around and your mouse to look around.

## Adding spectator points to the map
Spectator viewpoints are places in the world where the spectator can be teleported to.
These locations are defined in the map and cannot be changed during the simulation.
To create a new viewpoint, add a new `CameraActor` to the world and place it wherever you like.
Next, add a `AirsimSpectatorTeleportTrigger` to the world.
In the settings, set the camera to the one you just created.
Whenever the vehicle touches this trigger object, the spectator will be teleported to the selected camera (in follow-car mode).
