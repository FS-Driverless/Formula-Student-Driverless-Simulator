# Spectator
The spectator provides an eye into the virtual world from an external computer.
Through a network connection the virtual world state is replicated and shown on the screen.
The user can operate the camera with its mouse and keyboard.

## 1. Run the simulation in server mode
Download the packaged version of the unreal game and run it using:
```
FSDS.exe /Game/TrainingMap?listen -log PORT=7777
```
This will launch the simulation as a listen server.
This means that the game will run as normal but external clients are welcome to connect.

If you want to give access to clients from external computers you must ensure the specified port is accessible. 
In most cases you will have to add a firewall rule to allow the traffic.
When behind a router you might need to do some port forwarding.
Both TCP and UDP traffic must be able to travel from client to server on the configured port.

## 2. Running the spectator
Now we need to run the simulation on another computer pointing at the server.
You must use the same version of binaries as the server.
Run the simulation as follows:

```
FSDS.exe 0.0.0.0 -log PORT=7777
```
Where `0.0.0.0` is replaced by the external ip of the server.

If the connection fails the game willback to running as a normal simulation.
This fallback initiates after a connection timeout of 20 seconds.

Theoretically multiple simultaneous spectators in a single world should work but this is not tested or supported.

## 3. Using the spectator
Spectators can use the wsdaqe to move around and their mouse to look around. 
They can also use the buttons 1 til 9 to teleport to predefined camera locations.
We call these camera locations 'spectator points'. 

## 4. Adding spectator points to the map
Spectator points are places in the world where the spectator can teleport to using the number keys on the keyboard.
These locations are defined in the map and cannot be changed during the game.
To create a new point, add a new CameraActor to the world and add the `spectator` tag to the actor.
The AirsimSpectatorPawn will find them automatically and add them in the order that they were found.
