#!/usr/bin/env python

from flask import Flask, request, abort, render_template, jsonify
import subprocess, time, signal, sys, os, errno, json
from datetime import datetime
from threading import Timer

import sys
sys.path.append('../AirSim/PythonClient')
import airsim.client as airsim

app = Flask(__name__)
interfaceprocess = None

client = airsim.CarClient()
client.confirmConnection()
car_controls = airsim.CarControls()
ref =  client.getRefereeState()

doo_count = ref.doo_counter
lap_times = ref.laps
logs = []

with open('../config/team_config.json', 'r') as file:
    team_config = json.load(file)
    access_token = team_config['access_token']

@app.route('/', methods=['GET'])
def index():
    return render_template('index.html', teams=team_config['teams'], logs=logs)

@app.route('/mission/start', methods=['POST'])
def mission_start():
    # Abort if access token is incorrect
    if request.json is not None and request.json['access_token'] != access_token:
        abort(403, description='Incorrect access token')

    # Abort if empty request
    if request.json is None or request.json['id'] is None or request.json['mission'] is None:
        abort(400, description='Empty request.')    

    # Get team config
    teamId = request.json['id']
    mission = request.json['mission']
    for obj in team_config['teams']: 
        if obj['id'] == teamId: team = obj

    # Set ROS MASTER
    procenv = os.environ.copy()
    procenv["ROS_MASTER_URI"] = team['master']

    # Launch ROS bridge
    global interfaceprocess, log_file
    interfaceprocess = subprocess.Popen(['roslaunch', 'fsds_ros_bridge', 'fsds_ros_bridge.launch', 'mission:={}'.format(mission)], env=procenv)  

    # Start referee state listener
    referee_state_listener() 

    # Create log message
    log = '{}: {}'.format(str(datetime.now()), 'Mission started.')
    logs.append(log)

    # Create log file. Create logs directory if it does not exist
    filename = 'logs/{}_{}_{}.txt'.format(team['name'].lower().replace(' ', '-'), mission, str(datetime.now().strftime("%d-%m-%Y_%H:%M:%S")))
    if not os.path.exists(os.path.dirname(filename)):
        try:
            os.makedirs(os.path.dirname(filename))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    
    # Write to log file
    log_file = open(filename, 'w')
    log_file.write(log + '\n')

    return {'response': log}

@app.route('/mission/stop', methods=['POST'])
def mission_stop():
    # Abort if access token is incorrect
    if request.json is not None and request.json['access_token'] != access_token:
        abort(403, description='Incorrect access token')

    # Abort if ROS bridge is not running
    if interfaceprocess is None:
        abort(400, description='No process running.')

    # Check if previous process is still running
    if interfaceprocess is not None and interfaceprocess.poll() is None:
        # Try to stop it gracefully. SIGINT is the equivilant to doing ctrl-c
        interfaceprocess.send_signal(signal.SIGINT)
        time.sleep(3)
        # Still running?
        if interfaceprocess.poll() is None:
            # Kill it with fire
            interfaceprocess.terminate()
            # Wait for it to finish
            interfaceprocess.wait()

        global interfaceprocess
        interfaceprocess = None

    # Stop referee state listener
    timer.cancel()

    # Brake car
    car_controls.brake = 1
    client.setCarControls(car_controls)
    car_controls.brake = 0 #remove brake

    # Create log message
    log = '{}: {}'.format(str(datetime.now()), 'Mission stopped.')
    logs.append(log)
    del logs[:] # Clear logs

    # Write logs to file
    log_file.write(log + '\n')
    log_file.close()

    return {'response': log}

@app.route('/mission/reset', methods=['POST'])
def mission_reset():
    # Abort if access token is incorrect
    if request.json is not None and request.json['access_token'] != access_token:
        abort(403, description='Incorrect access token')

    # Reset simulator
    client.reset()
    log = '{}: {}'.format(str(datetime.now()), 'Car reset.')

    if interfaceprocess is not None:
        # Create log message and write to file
        logs.append(log)
        log_file.write(log + '\n')

    return {'response': log}

@app.route('/logs', methods=['GET'])
def get_logs():
    return {'response': logs}

@app.errorhandler(400)
def bad_request(e):
    return jsonify(error=str(e)), 400

@app.errorhandler(403)
def unauthorized(e):
    return jsonify(error=str(e)), 403

def referee_state_listener():
    global doo_count, lap_times, timer

    timer = Timer(0.5, referee_state_listener)
    timer.start()
    ref = client.getRefereeState()

    if doo_count != ref.doo_counter:
        delta = ref.doo_counter - doo_count
        for d in range(doo_count + 1, doo_count + delta + 1):
            log = '{}: {}. {} {}'.format(str(datetime.now()), 'Cone hit', str(d), 'DOO cone(s).')
            logs.append(log)
            log_file.write(log + '\n')
        doo_count = ref.doo_counter

    if len(lap_times) != len(ref.laps):
        lap_times = ref.laps
        lap_count = len(lap_times)
        lap_time = lap_times[-1]
        log = '{}: {}'.format(str(datetime.now()), 'Lap ' + str(lap_count) + ' completed. Lap time: ' + str(round(lap_time, 3)) + ' s.')
        logs.append(log) 
        log_file.write(log + '\n')

if __name__ == '__main__':
    app.run()