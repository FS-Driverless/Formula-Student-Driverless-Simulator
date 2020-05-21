#!/usr/bin/env python

from flask import Flask, request, abort, render_template
import subprocess, time, signal, sys, os, json
from datetime import datetime
from threading import Timer

import sys
sys.path.append('../AirSim/PythonClient')
from airsim.client import CarClient

app = Flask(__name__)

# client = CarClient()
# client.confirmConnection()
# ref =  client.getRefereeState()
# doo_count = ref.doo_counter
# laps = ref.laps

interfaceprocess = None
logs = []

with open('../config/team_config.json', 'r') as file:
    team_config = json.load(file)

interfaceprocess = None
logs = []

@app.route('/', methods=['GET'])
def index():
    return render_template('index.html', team_config=team_config, logs=logs)

# curl --header "Content-Type: application/json" --request POST --data '{"master": "http://localhost:11311", "mission": "trackdrive"}' http://localhost:5000/mission/selectcar
@app.route('/mission/start', methods=['POST'])
def mission_start():
    if request.json is None or request.json['id'] is None or request.json['mission'] is None:
        return abort(400)    

    teamId = request.json['id']
    mission = request.json['mission']
    for obj in team_config: 
        if obj['id'] == teamId: master = obj['master']

    procenv = os.environ.copy()
    procenv["ROS_MASTER_URI"] = master

    global interfaceprocess
    interfaceprocess = subprocess.Popen(['roslaunch', 'fsds_ros_bridge', 'fsds_ros_bridge.launch', 'mission:={}'.format(mission)], env=procenv)   

    log = '{}: {}'.format(str(datetime.now()), 'Mission started')
    logs.append(log)

    return {'response': log}

@app.route('/mission/stop', methods=['POST'])
def mission_stop():
    # check if previous process is still running
    if interfaceprocess is not None and interfaceprocess.poll() is None:
        # try to stop it gracefully. SIGINT is the equivilant to doing ctrl-c
        interfaceprocess.send_signal(signal.SIGINT)
        time.sleep(3)
        # still running?
        if interfaceprocess.poll() is None:
            # kill it with fire
            interfaceprocess.terminate()
            # wait for it to finish
            interfaceprocess.wait()

    log = '{}: {}'.format(str(datetime.now()), 'Mission stopped')
    logs.append(log)

    return {'response': log}

@app.route('/mission/reset', methods=['POST'])
def mission_reset():
    client.reset()

    log = '{}: {}'.format(str(datetime.now()), 'Car reset')
    logs.append(log)

    return {'response': log}

@app.route('/logs', methods=['GET'])
def get_logs():
    return {'response': logs}

def referee_state_listener():
    Timer.repeat(2.0, referee_state_listener).start()
    ref = client.getRefereeState()

    if doo_count != ref.doo_counter:
        doo_count = ref.doo_counter
        log = '{}: {}. {} {}'.format(str(datetime.now()), 'Cone hit', doo_count, 'DOO cones')
        logs.append(log)

    if laps != ref.laps:
        laps = ref.laps
        log = '{}: {} {} {}'.format(str(datetime.now()), 'Lap completed. Completed', doo_count, 'laps')
        logs.append(log) 

if __name__ == '__main__':
    app.run()
    #referee_state_listener()