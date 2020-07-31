#!/usr/bin/env python

import json

def getconfig():
    params = json.dumps({"access_token": "1234567890"}).encode('utf8')
    req = urllib.request.Request("http://35.204.160.166/", data=params, headers={'content-type': 'application/json'})
    with urllib.request.urlopen(req) as response:
        return json.loads(url.read().decode())

config = getconfig()

if 'car_settings' not in config:
    print("Simulator not running.")
    return

print("Current team: " + config['name'])

# Write team specific car settings to settings.json
filename = '~/Formula-Student-Driverless-Simulator/settings.json'
with open(filename, 'w') as file:
    json.dump(config['car_settings'], file, sort_keys=True, indent=4, separators=(',', ': ')) 

print("export ROS_MASTER_URI=" + config['master'])
localip = socket.gethostbyname(socket.gethostname())
print("export ROS_IP=" + localip)