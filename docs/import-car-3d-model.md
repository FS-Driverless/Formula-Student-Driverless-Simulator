# How to import your car 3d model

Import your own 3d model of your formula student car into the world and use it during simulation!

## Preface (a friendly warning)
Without experience with blender and unreal engine this whole process is painfull.
There are waaaay to many options and buttons in these tools and it never does what you want to do.
If you want your 3d model to be added to this repository you can create a github ticket and ask verry nicely if someone wants to help.
Maybe you find someone with some experience that will help you.
Make sure you have your fbx model prepared beforehand (see below).

Are you a great warrior forged in the fires of a collapsing star seeking for the ultimate challenge?
Let me introduce you to the world of importing 3d model. 

![](images/goingonanadventure.gif)

## 1. Prepare your 3d model

First we have to get the 3d model prepared and exported to fbx format.
Most 3d editing software support this format.
This tutorial however only describes blender becuase it's free (yay) and relatively easy to use.

Build your car and get it to the following structure:

![](images/blender-structure.png)

There should be 1 object for the chassy and 4 child objects for the wheels.
You do not need to define bones, those will be auto-created when importing to unreal.
Give the wheels easy names because we will be using those names later on.

Note: the car should be facing the positive x axis in blender.

This is also the time to paint your car.
Add surfuces, create textures and go crazy with paint.
Export any textures to image format.

Before we export the model, check that all deltra transforms have been applied and [all scales are set to 1](https://blender.stackexchange.com/questions/31769/how-to-set-the-current-scale-to-1/31771).
Also don't forget to [recalculate normals](https://blender.stackexchange.com/a/153695/100133)

Export the 3d model to FBX format using all default settings.
Ensure you are only exporting meshes and armature.

## 2. Importing the model into unreal engine
Create a new folder in `AirSim Content/VehicleADV/Cars/` with the name of your car.
Use all defaults, except:

* Check Skeletal Mesh
* Import content type: Geometry Only
* Skeleton 'None'

Now you should see a mesh file, physics asset and skeleton.

## 3. Styling your model
Go into the mesh file to style your car. 
There are many ways to make your mesh fancy using materials and textures.
Good luck Googling.

## 4. Configuring the physics asset
The physics asset defines which part of the vehicle interacts with the world.
It is basically the collision model.

**If you are not planning to use this car in an online competition** then you can configure the bounding boxes as close to the mesh as you want.
Some key settings to keep in mind:

* Each wheel bone needs it's own physics body.
* All wheel bodies need to be Physics Type 'Kinematic'
* Only the wheels should touch the ground.

Take a look at `Suv_Pa` for an example:

![](images/ue-suv_pa.png)

**If you are planning to use this car in an online competition (FSOnline)** you must use create the bounding boxes exactly the following specification:

* Width: 100cm
* Length: 180cm
* Height: 50cm

In competition all cars will be equal in behaviour. 
So all bounding boxes must be equal.
Below picture shows how bounding boxes can be bigger or smaller than the mesh.

![](images/ue-equality.png)

All bounding boxes will be checked by:

1. placing each vehicle into the virtual world using a SkeletalMeshActor
2. Going into top view mode (Alt-J)
3. Enabling Collision visualisation (Alt-C)
4. Using the middle-mouse button measuring tool to check the width and length 

The following physics asset are examples of above specification:

* `AirSim/Content/VehicleAdv/Cars/TechnionCar/RacingTechnion_PA.uasset`
* `AirSim/Content/VehicleAdv/Cars/ReferenceCar/ReferenceCar_Pa.uasset`
* `AirSim/Content/VehicleAdv/Cars/SuvCar/RacingSuv_Pa.uasset`

# 5. Create an animation
Go into your car folder and create a new Animation Blueprint.
Choose `VehicleAnimInstance` as parent class and choose your cars skeleton as target.
Now make it look like this:

![](images/ue-anim.png)

# 6. Create a pawn
Go into your car folder and create create a new Blueprint Class with base class CarPawn.
This will be the actual pawn that will be controlled by the autonomous system.

In the components browser, select 'Mesh'.
Next, in the details pane, set 'Skeletal Mesh' to your car.
Set 'Anim Class' to the animation class you created in step 5.

In the components browser, select 'VehicleMovement'
Next, in the details pane:

* Set `Mass` to `255,0`
* In the `Wheel Setups` set the first two `Wheel Class` to `FormulaFrontWheel` and the last two `Wheel Class` to `FormulaBackWheel`
* In `Wheel Setups` set the `Bone Name` of each wheel to the corresponding bone name as found in the skeleton. From top to bottom they are front left, front right, back left, back right
* From Drag Coefficient onwards all settings should be exactly equal to the settings in `SuvCarPawn`. Have fun copy-pasting.

# 7. Selecting a car
Update your settings.json `PawnPaths -> DefaultCar -> PawnBP` to point at the newly created pawn. 
It should (tm) work.

# 8. Sharing your vehicle with the world!

Wouldn't it be amazing if other people could enjoy your team's vehicle 3d model as well?
You are welcome to create a pull request with your vehicle and it might be added to the base project repository!
Submit the new model as a pull request which is up to date with the current project state, with the vehicle model and pawn files added. Contribution will be given in the readme of the project.
Please note that only GPLv2 licensed material is accepted into this repository.
