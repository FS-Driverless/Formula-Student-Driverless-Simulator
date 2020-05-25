#!/usr/bin/env python
from flask import Flask, request, abort, render_template, jsonify
from flask_classful import FlaskView, route
from datetime import datetime
from threading import Timer
import subprocess, time, signal, sys, os, errno, json, sys
sys.path.append('../AirSim/PythonClient')
import airsim.client as airsim

app = Flask(__name__)

class WebServer(FlaskView):

    def __init__(self):
        self.interface_process = None
        self.timer = None
        self.log_file = None
        self.logs = []

        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.car_controls = airsim.CarControls()

        ref =  self.client.getRefereeState()
        self.doo_count = ref.doo_counter
        self.lap_times = ref.laps

        with open('../config/team_config.json', 'r') as file:
            self.team_config = json.load(file)
            self.access_token = self.team_config['access_token']

    @route('/', methods=['GET'])
    def index(self):
        return render_template('index.html', teams=self.team_config['teams'], logs=self.logs)

    @route('/mission/start', methods=['POST'])
    def mission_start(self):
        # Abort if access token is incorrect
        if request.json is not None and request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if empty request
        if request.json is None or not all(key in request.json for key in ['id', 'mission', 'access_token']):
            abort(400, description='Empty request.')    

        # Get team config
        teamId = request.json['id']
        mission = request.json['mission']
        for obj in self.team_config['teams']: 
            if obj['id'] == teamId: team = obj

        # Set ROS MASTER
        procenv = os.environ.copy()
        procenv["ROS_MASTER_URI"] = team['master']

        # Launch ROS bridge
        self.interface_process = subprocess.Popen(['roslaunch', 'fsds_ros_bridge', 'fsds_ros_bridge.launch', 'mission:={}'.format(mission)], env=procenv)  

        # Start referee state listener
        self.referee_state_listener() 

        # Create log message
        log = '{}: {}'.format(str(datetime.now()), 'Mission started.')
        self.logs.append(log)

        # Create log file. Create logs directory if it does not exist
        filename = 'logs/{}_{}_{}.txt'.format(team['name'].lower().replace(' ', '-'), mission, str(datetime.now().strftime("%d-%m-%Y_%H:%M:%S")))
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        
        # Write to log file
        self.log_file = open(filename, 'w')
        self.log_file.write(log + '\n')

        return {'response': log}

    @route('/mission/stop', methods=['POST'])
    def mission_stop(self):
        # Abort if access token is incorrect
        if request.json is not None and request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')   

        # Abort if ROS bridge is not running
        if self.interface_process is None:
            abort(400, description='No process running.')

        # Check if previous process is still running
        if self.interface_process is not None and self.interface_process.poll() is None:
            # Try to stop it gracefully. SIGINT is the equivilant to doing ctrl-c
            self.interface_process.send_signal(signal.SIGINT)
            time.sleep(3)
            # Still running?
            if self.interface_process.poll() is None:
                # Kill it with fire
                self.interface_process.terminate()
                # Wait for it to finish
                self.interface_process.wait()

            self.interface_process = None

        # Stop referee state listener
        self.timer.cancel()

        # Brake car
        self.car_controls.brake = 1
        self.client.setCarControls(self.car_controls)
        self.car_controls.brake = 0 #remove brake

        # Create log message
        log = '{}: {}'.format(str(datetime.now()), 'Mission stopped.')
        self.logs.append(log)
        del self.logs[:] # Clear logs

        # Write logs to file
        self.log_file.write(log + '\n')
        self.log_file.close()

        return {'response': log}

    @route('/mission/reset', methods=['POST'])
    def mission_reset(self):
        # Abort if access token is incorrect
        if request.json is not None and request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')  

        # Reset simulator
        #self.client.reset()
        log = '{}: {}'.format(str(datetime.now()), 'Car reset.')

        if self.interface_process is not None:
            # Create log message and write to file
            self.logs.append(log)
            self.log_file.write(log + '\n')

        return {'response': log}

    @route('/logs', methods=['POST'])
    def get_logs(self):
        # Abort if access token is incorrect
        if request.json is not None and request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')  

        return {'response': self.logs}

    def referee_state_listener(self):
        self.timer = Timer(0.5, self.referee_state_listener)
        self.timer.start()
        ref = self.client.getRefereeState()

        if self.doo_count != ref.doo_counter:
            delta = ref.doo_counter - self.doo_count

            for d in range(self.doo_count + 1, self.doo_count + delta + 1):
                log = '{}: {}. {} {}'.format(str(datetime.now()), 'Cone hit', str(d), 'DOO cone(s).')
                self.logs.append(log)
                self.log_file.write(log + '\n')

            self.doo_count = ref.doo_counter

        if len(self.lap_times) != len(ref.laps):
            lap_times = ref.laps
            lap_count = len(lap_times)
            lap_time = lap_times[-1]
            log = '{}: {}'.format(str(datetime.now()), 'Lap ' + str(lap_count) + ' completed. Lap time: ' + str(round(lap_time, 3)) + ' s.')
            self.logs.append(log) 
            self.log_file.write(log + '\n')

@app.errorhandler(400)
def bad_request(e):
    return jsonify(error=str(e)), 400

@app.errorhandler(403)
def unauthorized(e):
    return jsonify(error=str(e)), 403

WebServer.register(app, route_base='/')

if __name__ == '__main__':
    app.run()