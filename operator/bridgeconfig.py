#!/usr/bin/env python3
import os
import socket
import json
import urllib.request as request

# this script is supposed to be evaluatd in a bash terminal like so;
# eval `./bridgeconfig.py`

operator_url = "http://10.164.0.3:5000"
operator_token = "1234567890"

def getconfig():
    params = json.dumps({"access_token": operator_token}).encode('utf8')
    req = request.Request(operator_url+"/config", data=params, headers={'content-type': 'application/json'})
    with request.urlopen(req) as response:
        return json.loads(response.read().decode())

config = getconfig()

if 'team' not in config:
    print("echo Simulator not running.")
    exit()

print("echo Current team: " + config['team']['name'] + ";")

# Write team specific car settings to settings.json
filename = os.path.expanduser('~/Formula-Student-Driverless-Simulator/settings.json')
with open(filename, 'w') as file:
    json.dump(config['team']['car_settings'], file, sort_keys=True, indent=4, separators=(',', ': '))

masteruri = config['team']['master']
mission = config['mission']
track = config['track']

print("export ROS_MASTER_URI=" + masteruri + ";")
print("echo set ROS_MASTER_URI to " + masteruri + ";")

localip = socket.gethostbyname(socket.gethostname())
print("export ROS_IP=" + localip + ";")
print("echo set ROS_IP to " + localip + ";")

print("export OPERATOR_TOKEN="+ operator_token +";")
print("echo 'set OPERATOR_TOKEN to *****';")

print("export OPERATOR_URL=" + operator_url + ";")
print("echo set OPERATOR_URL to " + operator_url + ";")

print("echo mission: " + mission + ";")
print("echo track: " + track + ";")

print("roslaunch fsds_ros_bridge fsds_ros_bridge.launch competition_mode:=false mission_name:=" + mission + " host:=simulator track_name:=" + track)
