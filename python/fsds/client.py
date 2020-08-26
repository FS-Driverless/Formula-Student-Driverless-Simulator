from __future__ import print_function

from .utils import *
from .types import *

import msgpackrpc # pip install msgpack-rpc-python
import numpy as np # pip install numpy
import msgpack
import time
import math
import logging

class FSDSClient:
    def __init__(self, ip = "", port = 41451, timeout_value = 3):
        if (ip == ""):
            ip = "127.0.0.1"
        self.client = msgpackrpc.Client(msgpackrpc.Address(ip, port), timeout = timeout_value, pack_encoding = 'utf-8', unpack_encoding = 'utf-8')

    # -----------------------------------  Common vehicle APIs ---------------------------------------------
    def reset(self):
        """
        Reset the vehicle to its original starting state

        Note that you must call `enableApiControl` and `armDisarm` again after the call to reset
        """
        self.client.call('reset')

    def ping(self):
        """
        If connection is established then this call will return true otherwise it will be blocked until timeout

        Returns:
            bool:
        """
        return self.client.call('ping')

    def getClientVersion(self):
        return 1 # sync with C++ client

    def getServerVersion(self):
        return self.client.call('getServerVersion')

    def getMinRequiredServerVersion(self):
        return 1 # sync with C++ client

    def getMinRequiredClientVersion(self):
        return self.client.call('getMinRequiredClientVersion')

    def enableApiControl(self, is_enabled, vehicle_name = 'FSCar'):
        """
        Enables or disables API control for vehicle corresponding to vehicle_name

        Args:
            is_enabled (bool): True to enable, False to disable API control
            vehicle_name (str, optional): Name of the vehicle to send this command to
        """
        self.client.call('enableApiControl', is_enabled, vehicle_name)

    def isApiControlEnabled(self, vehicle_name = 'FSCar'):
        """
        Returns true if API control is established.

        If false (which is default) then API calls would be ignored. After a successful call to `enableApiControl`, `isApiControlEnabled` should return true.

        Args:
            vehicle_name (str, optional): Name of the vehicle

        Returns:
            bool: If API control is enabled
        """
        return self.client.call('isApiControlEnabled', vehicle_name)

    def armDisarm(self, arm, vehicle_name = 'FSCar'):
        """
        Arms or disarms vehicle

        Args:
            arm (bool): True to arm, False to disarm the vehicle
            vehicle_name (str, optional): Name of the vehicle to send this command to

        Returns:
            bool: Success
        """
        return self.client.call('armDisarm', arm, vehicle_name)

    def simPause(self, is_paused):
        """
        Pauses simulation

        Args:
            is_paused (bool): True to pause the simulation, False to release
        """
        self.client.call('simPause', is_paused)

    def simIsPause(self):
        """
        Returns true if the simulation is paused

        Returns:
            bool: If the simulation is paused
        """
        return self.client.call("simIsPaused")

    def simContinueForTime(self, seconds):
        """
        Continue the simulation for the specified number of seconds

        Args:
            seconds (float): Time to run the simulation for
        """
        self.client.call('simContinueForTime', seconds)

    def getHomeGeoPoint(self, vehicle_name = 'FSCar'):
        """
        Get the Home location of the vehicle

        Args:
            vehicle_name (str, optional): Name of vehicle to get home location of

        Returns:
            GeoPoint: Home location of the vehicle
        """
        return GeoPoint.from_msgpack(self.client.call('getHomeGeoPoint', vehicle_name))

    def confirmConnection(self):
        """
        Checks state of connection every 1 sec and reports it in Console so user can see the progress for connection.
        If ping fails the program is exited.
        """
        if self.ping():
            print("Ping to simulator OK")
        else:
             print("Ping to simulator FAIL")
             exit(1)

    def simSwapTextures(self, tags, tex_id = 0, component_id = 0, material_id = 0):
        """
        Runtime Swap Texture API

        See https://microsoft.github.io/AirSim/retexturing/ for details

        Args:
            tags (str): string of "," or ", " delimited tags to identify on which actors to perform the swap
            tex_id (int, optional): indexes the array of textures assigned to each actor undergoing a swap

                                    If out-of-bounds for some object's texture set, it will be taken modulo the number of textures that were available
            component_id (int, optional):
            material_id (int, optional):

        Returns:
            list[str]: List of objects which matched the provided tags and had the texture swap perfomed
        """
        return self.client.call("simSwapTextures", tags, tex_id, component_id, material_id)

    # time-of-day control
    def simSetTimeOfDay(self, is_enabled, start_datetime = "", is_start_datetime_dst = False, celestial_clock_speed = 1, update_interval_secs = 60, move_sun = True):
        """
        Control the position of Sun in the environment

        Sun's position is computed using the coordinates specified in `OriginGeopoint` in settings for the date-time specified in the argument,
        else if the string is empty, current date & time is used

        Args:
            is_enabled (bool): True to enable time-of-day effect, False to reset the position to original
            start_datetime (str, optional): Date & Time in %Y-%m-%d %H:%M:%S format, e.g. `2018-02-12 15:20:00`
            is_start_datetime_dst (bool, optional): True to adjust for Daylight Savings Time
            celestial_clock_speed (float, optional): Run celestial clock faster or slower than simulation clock
                                                     E.g. Value 100 means for every 1 second of simulation clock, Sun's position is advanced by 100 seconds
                                                     so Sun will move in sky much faster
            update_interval_secs (float, optional): Interval to update the Sun's position
            move_sun (bool, optional): Whether or not to move the Sun
        """
        self.client.call('simSetTimeOfDay', is_enabled, start_datetime, is_start_datetime_dst, celestial_clock_speed, update_interval_secs, move_sun)

    # weather
    def simEnableWeather(self, enable):
        """
        Enable Weather effects. Needs to be called before using `simSetWeatherParameter` API

        Args:
            enable (bool): True to enable, False to disable
        """
        self.client.call('simEnableWeather', enable)

    def simSetWeatherParameter(self, param, val):
        """
        Enable various weather effects

        Args:
            param (WeatherParameter): Weather effect to be enabled
            val (float): Intensity of the effect, Range 0-1
        """
        self.client.call('simSetWeatherParameter', param, val)

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the ImageType members
    def simGetImage(self, camera_name, image_type, vehicle_name = 'FSCar'):
        """
        Get a single image

        Returns bytes of png format image which can be dumped into abinary file to create .png image
        `string_to_uint8_array()` can be used to convert into Numpy unit8 array
        See https://microsoft.github.io/AirSim/image_apis/ for details

        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            image_type (ImageType): Type of image required
            vehicle_name (str, optional): Name of the vehicle with the camera

        Returns:
            Binary string literal of compressed png image
        """
        # todo: in future remove below, it's only for compatibility to pre v1.2
        camera_name = str(camera_name)

        # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
        result = self.client.call('simGetImage', camera_name, image_type, vehicle_name)
        if (result == "" or result == "\0"):
            return None
        return result

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the ImageType members
    def simGetImages(self, requests, vehicle_name = 'FSCar'):
        """
        Get multiple images

        See https://microsoft.github.io/AirSim/image_apis/ for details and examples

        Args:
            requests (list[ImageRequest]): Images required
            vehicle_name (str, optional): Name of vehicle associated with the camera

        Returns:
            list[ImageResponse]:
        """
        responses_raw = self.client.call('simGetImages', requests, vehicle_name)
        return [ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]

    # gets the static meshes in the unreal scene
    def simGetMeshPositionVertexBuffers(self):
        """
        Returns the static meshes that make up the scene

        See https://microsoft.github.io/AirSim/meshes/ for details and how to use this

        Returns:
            list[MeshPositionVertexBuffersResponse]:
        """
        responses_raw = self.client.call('simGetMeshPositionVertexBuffers')
        return [MeshPositionVertexBuffersResponse.from_msgpack(response_raw) for response_raw in responses_raw]

    def simGetCollisionInfo(self, vehicle_name = 'FSCar'):
        """
        Args:
            vehicle_name (str, optional): Name of the Vehicle to get the info of

        Returns:
            CollisionInfo:
        """
        return CollisionInfo.from_msgpack(self.client.call('simGetCollisionInfo', vehicle_name))

    def simSetVehiclePose(self, pose, ignore_collison, vehicle_name = 'FSCar'):
        """
        Set the pose of the vehicle

        If you don't want to change position (or orientation) then just set components of position (or orientation) to floating point nan values

        Args:
            pose (Pose): Desired Pose pf the vehicle
            ignore_collision (bool): Whether to ignore any collision or not
            vehicle_name (str, optional): Name of the vehicle to move
        """
        self.client.call('simSetVehiclePose', pose, ignore_collison, vehicle_name)

    def simGetVehiclePose(self, vehicle_name = 'FSCar'):
        """
        Args:
            vehicle_name (str, optional): Name of the vehicle to get the Pose of

        Returns:
            Pose:
        """
        pose = self.client.call('simGetVehiclePose', vehicle_name)
        return Pose.from_msgpack(pose)

    def simSetTraceLine(self, color_rgba, thickness=1.0, vehicle_name = 'FSCar'):
        """
        Modify the color and thickness of the line when Tracing is enabled

        Tracing can be enabled by pressing T in the Editor or setting `EnableTrace` to `True` in the Vehicle Settings

        Args:
            color_rgba (list): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of the line
            vehicle_name (string, optional): Name of the vehicle to set Trace line values for
        """
        self.client.call('simSetTraceLine', color_rgba, thickness, vehicle_name)

    def simGetObjectPose(self, object_name):
        """
        Args:
            object_name (str): Object to get the Pose of

        Returns:
            Pose:
        """
        pose = self.client.call('simGetObjectPose', object_name)
        return Pose.from_msgpack(pose)

    def simSetObjectPose(self, object_name, pose, teleport = True):
        """
        Set the pose of the object(actor) in the environment

        The specified actor must have Mobility set to movable, otherwise there will be undefined behaviour.
        See https://www.unrealengine.com/en-US/blog/moving-physical-objects for details on how to set Mobility and the effect of Teleport parameter

        Args:
            object_name (str): Name of the object(actor) to move
            pose (Pose): Desired Pose of the object
            teleport (bool, optional): Whether to move the object immediately without affecting their velocity

        Returns:
            bool: If the move was successful
        """
        return self.client.call('simSetObjectPose', object_name, pose, teleport)

    def simListSceneObjects(self, name_regex = '.*'):
        """
        Lists the objects present in the environment

        Default behaviour is to list all objects, regex can be used to return smaller list of matching objects or actors

        Args:
            name_regex (str, optional): String to match actor names against, e.g. "Cylinder.*"

        Returns:
            list[str]: List containing all the names
        """
        return self.client.call('simListSceneObjects', name_regex)

    def simSetSegmentationObjectID(self, mesh_name, object_id, is_name_regex = False):
        """
        Set segmentation ID for specific objects

        See https://microsoft.github.io/AirSim/image_apis/#segmentation for details

        Args:
            mesh_name (str): Name of the mesh to set the ID of (supports regex)
            object_id (int): Object ID to be set, range 0-255

                             RBG values for IDs can be seen at https://microsoft.github.io/AirSim/seg_rgbs.txt
            is_name_regex (bool, optional): Whether the mesh name is a regex

        Returns:
            bool: If the mesh was found
        """
        return self.client.call('simSetSegmentationObjectID', mesh_name, object_id, is_name_regex)

    def simGetSegmentationObjectID(self, mesh_name):
        """
        Returns Object ID for the given mesh name

        Mapping of Object IDs to RGB values can be seen at https://microsoft.github.io/AirSim/seg_rgbs.txt

        Args:
            mesh_name (str): Name of the mesh to get the ID of
        """
        return self.client.call('simGetSegmentationObjectID', mesh_name)

    def simPrintLogMessage(self, message, message_param = "", severity = 0):
        """
        Prints the specified message in the simulator's window.

        If message_param is supplied, then it's printed next to the message and in that case if this API is called with same message value
        but different message_param again then previous line is overwritten with new line (instead of API creating new line on display).

        For example, `simPrintLogMessage("Iteration: ", to_string(i))` keeps updating same line on display when API is called with different values of i.
        The valid values of severity parameter is 0 to 3 inclusive that corresponds to different colors.

        Args:
            message (str): Message to be printed
            message_param (str, optional): Parameter to be printed next to the message
            severity (int, optional): Range 0-3, inclusive, corresponding to the severity of the message
        """
        self.client.call('simPrintLogMessage', message, message_param, severity)

    def simGetCameraInfo(self, camera_name, vehicle_name = 'FSCar'):
        """
        Get details about the camera

        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            vehicle_name (str, optional): Vehicle which the camera is associated with

        Returns:
            CameraInfo:
        """
        # TODO: below str() conversion is only needed for legacy reason and should be removed in future
        return CameraInfo.from_msgpack(self.client.call('simGetCameraInfo', str(camera_name), vehicle_name))

    def simSetCameraOrientation(self, camera_name, orientation, vehicle_name = 'FSCar'):
        """
        - Control the orientation of a selected camera

        Args:
            camera_name (str): Name of the camera to be controlled
            orientation (Quaternionr): Quaternion representing the desired orientation of the camera
            vehicle_name (str, optional): Name of vehicle which the camera corresponds to
        """
        # TODO: below str() conversion is only needed for legacy reason and should be removed in future
        self.client.call('simSetCameraOrientation', str(camera_name), orientation, vehicle_name)

    def simSetCameraFov(self, camera_name, fov_degrees, vehicle_name = 'FSCar'):
        """
        - Control the field of view of a selected camera

        Args:
            camera_name (str): Name of the camera to be controlled
            fov_degrees (float): Value of field of view in degrees
            vehicle_name (str, optional): Name of vehicle which the camera corresponds to
        """
        # TODO: below str() conversion is only needed for legacy reason and should be removed in future
        self.client.call('simSetCameraFov', str(camera_name), fov_degrees, vehicle_name)

    def simGetGroundTruthKinematics(self, vehicle_name = 'FSCar'):
        """
        Get Ground truth kinematics of the vehicle

        Args:
            vehicle_name (str, optional): Name of the vehicle

        Returns:
            KinematicsState: Ground truth of the vehicle
        """
        kinematics_state = self.client.call('simGetGroundTruthKinematics', vehicle_name)
        return KinematicsState.from_msgpack(kinematics_state)
    simGetGroundTruthKinematics.__annotations__ = {'return': KinematicsState}

    # sensor APIs
    def getImuData(self, imu_name = '', vehicle_name = 'FSCar'):
        """
        Args:
            imu_name (str, optional): Name of IMU to get data from, specified in settings.json. When no name is provided the last imu will be used.
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            ImuData:
        """
        return ImuData.from_msgpack(self.client.call('getImuData', imu_name, vehicle_name))

    def getGpsData(self, gps_name = '', vehicle_name = 'FSCar'):
        """
        Args:
            gps_name (str, optional): Name of GPS to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            GpsData:
        """
        return GpsData.from_msgpack(self.client.call('getGpsData', gps_name, vehicle_name))

    def getGroundSpeedSensorData(self, vehicle_name = 'FSCar'):
        """
        Args:
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to. Default FSCar.
        Returns:
            GroundSpeedSensorData:
        """
        return GroundSpeedSensorData.from_msgpack(self.client.call('getGroundSpeedSensorData', vehicle_name))

    def getDistanceSensorData(self, distance_sensor_name = '', vehicle_name = 'FSCar'):
        """
        Args:
            distance_sensor_name (str, optional): Name of Distance Sensor to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            DistanceSensorData:
        """
        return DistanceSensorData.from_msgpack(self.client.call('getDistanceSensorData', distance_sensor_name, vehicle_name))

    def getLidarData(self, lidar_name = '', vehicle_name = 'FSCar'):
        """
        Args:
            lidar_name (str, optional): Name of Lidar to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to

        Returns:
            LidarData:
        """
        return LidarData.from_msgpack(self.client.call('getLidarData', lidar_name, vehicle_name))

    def simGetLidarSegmentation(self, lidar_name = '', vehicle_name = 'FSCar'):
        """
        Returns Segmentation ID of each point's collided object in the last Lidar update

        Args:
            lidar_name (str, optional): Name of Lidar sensor
            vehicle_name (str, optional): Name of the vehicle wth the sensor

        Returns:
            list[int]: Segmentation IDs of the objects
        """
        return self.client.call('simGetLidarSegmentation', lidar_name, vehicle_name)

    #  Plotting APIs
    def simFlushPersistentMarkers(self):
        """
        Clear any persistent markers - those plotted with setting `is_persistent=True` in the APIs below
        """
        self.client.call('simFlushPersistentMarkers')

    def simPlotPoints(self, points, color_rgba=[1.0, 0.0, 0.0, 1.0], size = 10.0, duration = -1.0, is_persistent = False):
        """
        Plot a list of 3D points in World NED frame

        Args:
            points (list[Vector3r]): List of Vector3r objects
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            size (float, optional): Size of plotted point
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call('simPlotPoints', points, color_rgba, size, duration, is_persistent)

    def simPlotLineStrip(self, points, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness = 5.0, duration = -1.0, is_persistent = False):
        """
        Plots a line strip in World NED frame, defined from points[0] to points[1], points[1] to points[2], ... , points[n-2] to points[n-1]

        Args:
            points (list[Vector3r]): List of 3D locations of line start and end points, specified as Vector3r objects
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of line
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call('simPlotLineStrip', points, color_rgba, thickness, duration, is_persistent)

    def simPlotLineList(self, points, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness = 5.0, duration = -1.0, is_persistent = False):
        """
        Plots a line strip in World NED frame, defined from points[0] to points[1], points[2] to points[3], ... , points[n-2] to points[n-1]

        Args:
            points (list[Vector3r]): List of 3D locations of line start and end points, specified as Vector3r objects. Must be even
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of line
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call('simPlotLineList', points, color_rgba, thickness, duration, is_persistent)

    def simPlotArrows(self, points_start, points_end, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness = 5.0, arrow_size = 2.0, duration = -1.0, is_persistent = False):
        """
        Plots a list of arrows in World NED frame, defined from points_start[0] to points_end[0], points_start[1] to points_end[1], ... , points_start[n-1] to points_end[n-1]

        Args:
            points_start (list[Vector3r]): List of 3D start positions of arrow start positions, specified as Vector3r objects
            points_end (list[Vector3r]): List of 3D end positions of arrow start positions, specified as Vector3r objects
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of line
            arrow_size (float, optional): Size of arrow head
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call('simPlotArrows', points_start, points_end, color_rgba, thickness, arrow_size, duration, is_persistent)


    def simPlotStrings(self, strings, positions, scale = 5, color_rgba=[1.0, 0.0, 0.0, 1.0], duration = -1.0):
        """
        Plots a list of strings at desired positions in World NED frame.

        Args:
            strings (list[String], optional): List of strings to plot
            positions (list[Vector3r]): List of positions where the strings should be plotted. Should be in one-to-one correspondence with the strings' list
            scale (float, optional): Font scale of transform name
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            duration (float, optional): Duration (seconds) to plot for
        """
        self.client.call('simPlotStrings', strings, positions, scale, color_rgba, duration)

    def simPlotTransforms(self, poses, scale = 5.0, thickness = 5.0, duration = -1.0, is_persistent = False):
        """
        Plots a list of transforms in World NED frame.

        Args:
            poses (list[Pose]): List of Pose objects representing the transforms to plot
            scale (float, optional): Length of transforms' axes
            thickness (float, optional): Thickness of transforms' axes
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.call('simPlotTransforms', poses, scale, thickness, duration, is_persistent)

    def simPlotTransformsWithNames(self, poses, names, tf_scale = 5.0, tf_thickness = 5.0, text_scale = 10.0, text_color_rgba = [1.0, 0.0, 0.0, 1.0], duration = -1.0):
        """
        Plots a list of transforms with their names in World NED frame.

        Args:
            poses (list[Pose]): List of Pose objects representing the transforms to plot
            names (list[string]): List of strings with one-to-one correspondence to list of poses
            tf_scale (float, optional): Length of transforms' axes
            tf_thickness (float, optional): Thickness of transforms' axes
            text_scale (float, optional): Font scale of transform name
            text_color_rgba (list, optional): desired RGBA values from 0.0 to 1.0 for the transform name
            duration (float, optional): Duration (seconds) to plot for
        """
        self.client.call('simPlotTransformsWithNames', poses, names, tf_scale, tf_thickness, text_scale, text_color_rgba, duration)

    def cancelLastTask(self, vehicle_name = 'FSCar'):
        """
        Cancel previous Async task

        Args:
            vehicle_name (str, optional): Name of the vehicle
        """
        self.client.call('cancelLastTask', vehicle_name)

    def waitOnLastTask(self, timeout_sec = float('nan')):
        """
        Wait for the last Async task to complete

        Args:
            timeout_sec (float, optional): Time for the task to complete

        Returns:
            bool: Result of the last task

                  True if the task completed without cancellation or timeout
        """
        return self.client.call('waitOnLastTask', timeout_sec)

    def setCarControls(self, controls, vehicle_name = 'FSCar'):
        """
        Control the car using throttle, steering, brake, etc.

        Args:
            controls (CarControls): Struct containing control values
            vehicle_name (str, optional): Name of vehicle to be controlled
        """
        self.client.call('setCarControls', controls, vehicle_name)

    def getCarState(self, vehicle_name = 'FSCar'):
        """
        Args:
            vehicle_name (str, optional): Name of vehicle

        Returns:
            CarState:
        """
        state_raw = self.client.call('getCarState', vehicle_name)
        return CarState.from_msgpack(state_raw)

    def getCarControls(self, vehicle_name = 'FSCar'):
        """
        Args:
            vehicle_name (str, optional): Name of vehicle

        Returns:
            CarControls:
        """
        controls_raw = self.client.call('getCarControls', vehicle_name)
        return CarControls.from_msgpack(controls_raw)

    def getRefereeState(self):
        referee_state_raw = self.client.call('getRefereeState')
        return RefereeState.from_msgpack(referee_state_raw)