# Vehicle Dynamic model

One of the most controversial subjects of any competition simulator is the vehicle dynamic model. This is the piece of the simulation that actually changes the state of the vehicle. In building this simulator for the FSOnline competition, our design philosophy was the following:

* **All teams will use the same vehicle dynamic model**. We are well aware that all teams have put effort into developing dynamic models of their own FSCar for controls and simulation purposes. However, we want the FSOnline DV Dynamic event to purely be a battle of autonomous software. Even if this will require teams to tweak their path planning and control algorithms, it will make sure that the winner of this event is truly the team that can take a grey box race car system and push it to its limits the most.

* **The dynamic model will have a high enough fidelity such that it is virtually impossible to overfit to it/reverse the plant or run open loop**. This will force teams to use system identification techniques similar to the ones that are used on a real car and no cheating or unfair advantage will be given to any teams.

* **A third-party, open-source model would be ideal**. This way, not even the developers of the simulation (Formula Student Team Delft) would have an edge over other teams. Everyone has access to the same code and has had no experience working with it or been involved in developing it.


The Unreal Engine repository contains (as a third party library) the code for [PhysXVehicles](https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Vehicles.html) that was developed by Nvidia. This seemed like the perfect solution for our simulator, given that it complies with all the criteria of our design philosophy above. Airsim simply interacts with the PhysXCar API in [these](https://github.com/FS-Online/Driverless-Competition-Simulator/tree/93fa784106c70e53e973e6d755ab5430feeef9da/UE4Project/Plugins/AirSim/Source/Vehicles/Car) files, which have configurable parameters exposed in the uassets files [here](https://github.com/FS-Online/Driverless-Competition-Simulator/tree/master/UE4Project/Plugins/AirSim/Content/VehicleAdv/SUV). 

These are the current (high-level) vehicle parameters (you will be notified if anything changes):

* Mass: 255 kg
* Max speed: ~27 m/s
* Drag coefficient: 0.3 [-]
* Chassis width: 1.44 m
* Chassis height: 1.22 m

These parameters can only be found when opening the UE4Editor and opening the SUVCarPawn.uasset file located [here](https://github.com/FS-Online/Driverless-Competition-Simulator/tree/master/UE4Project/Plugins/AirSim/Content/VehicleAdv/SUV).

![SUVCarPawn](images/vehicle_dynamic_model.png)