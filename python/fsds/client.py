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

        Note that you must call `enableApiControl` again after the call to reset
        """
        self.client.call('reset')

    def ping(self):
        """
        If connection is established then this call will return true otherwise it will be blocked until timeout

        Returns:
            bool:
        """
        return self.client.call('ping')

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
    def getLidarData(self, lidar_name = '', vehicle_name = 'FSCar'):
        """
        Args:
            lidar_name (str, optional): Name of Lidar to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to
        Returns:
            LidarData:
        """
        return LidarData.from_msgpack(self.client.call('getLidarData', lidar_name, vehicle_name))

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

    def getRefereeState(self):
        referee_state_raw = self.client.call('getRefereeState')
        return RefereeState.from_msgpack(referee_state_raw)

    def getSettingsString(self):
        """
        Fetch the settings text being used by AirSim
        Returns:
            str: Settings text in JSON format
        """
        return self.client.call('getSettingsString')
