## Competition system flow and Signals

### Staging
At some point, your AS will be staged: The vehicle is placed prior to the starting line, the `fsds_ros_bridge` node will connect to the ROS system.
From this point onwards, the AS will receive sensor data and can control the vehicle by publishing vehicle setpoints.
However, it shouldn't start driving the vehicle just yet!

### Starting 
Just like with a physical FS event, the vehicle must only start driving after a GO signal is received.
The GO signal indicates that the AS must start the mission and that the vehicle should start driving.
This signal is also known as the 'green flag' signal or 'starting' signal. Within this repo, we will reference it as 'GO'.
Within the GO signal, the mission and track are added. 
Currently, `autocross` and `trackdrive` are the supported missions.

In autocross, the vehicle has to complete a single lap on an unknown track.
On the trackdrive, the vehicle has to finish 10 laps on a track it has previously seen.
During the competition, on each track, every AS will do an autocross mission before a trackdrive mission.
It can occur that multiple autocross runs on different tracks take place before going to trackdrive.
It can also happen that multiple autocross runs take place on the same track.
For example, the AS might be requested to do:

1. autocross on track A
2. autocross on track B
3. trackdrive on track A
4. autocross on track C
5. autocross on track C (re-run)
6. trackdrive on track C

The AS must implement the following behaviour:

* When the AS is requested to do autocross on a track that it has seen before, it must delete any and all data it gathered during all previous runs on this track.
* When the AS is requested to do trackdrive on a track where it has done a trackdrive previously, it must delete any and all data it gathered during all previous trackdrive runs on this track. However, the data gathered during the last autocross on this track shouldn't be deleted.

An exception to this rule is data recorded with the exclusive intent to analyze the AS's behaviour after the event.
This includes all files that the AS only writes to but does not read from.

To make the AS aware of which track it is driving, the GO signal includes a unique identifier of the upcoming track.

After the initial GO signal, the signal is continuously re-sent at 1 Hz to ensure it arrives at the team's AS.
The timestamp of all consecutive GO signals is equal to the first one.

### Finishing
There are two ways to conclude a run: finishing or stopping.

When the autonomous system feels it concluded it's run, it should send a FINISHED signal.
The FINISHED signal tells the simulator that the AS no longer wants to control the vehicle.
As such, the simulator will stop the `fsds_ros_bridge` and the AS will no longer receive sensor data or be able to control the vehicle.

When the official decides that the run is over it will stop the simulation.
See the rulebook for a description of when the official does so.
When the simulation is stopped the `fsds_ros_bridge` is stopped immediately and the AS will no longer receive sensor data or be able to control the vehicle.
The AS will not receive a signal that this happened.
To detect a stop, the AS should keep an eye on the GO signal.

The general rule is: If the AS did not receive a GO signal for 4 seconds the AS can assume the `fsds_ros_bridge` is stopped.
When this state is detected, the AS can reset itself and prepare for the next mission.


## Ros messages

The AS will receive the GO signal on the following topic:

- `/fsds/signal/go`

And it AS can publish the FINISHED on this topic:

- `/fsds/signal/finished`

The AS must publish vehicle control commands on this topic:

- `/fsds/control_command`

Read more about the techincal detalis of these topics in the [ros-bridge documentation](ros-bridge.md)