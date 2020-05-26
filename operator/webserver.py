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

    interface_process = None
    timer = None
    log_file = None
    logs = []

    client = airsim.CarClient()
    client.confirmConnection()
    car_controls = airsim.CarControls()

    ref =  client.getRefereeState()
    doo_count = ref.doo_counter
    lap_times = ref.laps

    def __init__(self):
        with open('../config/team_config.json', 'r') as file:
            self.team_config = json.load(file)
            self.access_token = self.team_config['access_token']

    @route('/', methods=['GET'])
    def index(self):
        return render_template('index.html', teams=self.team_config['teams'], logs=WebServer.logs)

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
        WebServer.interface_process = subprocess.Popen(['roslaunch', 'fsds_ros_bridge', 'fsds_ros_bridge.launch', 'mission:={}'.format(mission)], env=procenv)  

        # Start referee state listener
        self.referee_state_listener() 

        # Create log message
        log = '{}: {}'.format(str(datetime.now()), 'Mission started.')
        WebServer.logs.append(log)

        # Create log file. Create logs directory if it does not exist
        filename = 'logs/{}_{}_{}.txt'.format(team['name'].lower().replace(' ', '-'), mission, str(datetime.now().strftime("%d-%m-%Y_%H:%M:%S")))
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        
        # Write to log file
        WebServer.log_file = open(filename, 'w')
        WebServer.log_file.write(log + '\n')

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
        if WebServer.interface_process is None:
            abort(400, description='No process running.')

        # Check if previous process is still running
        if WebServer.interface_process is not None and WebServer.interface_process.poll() is None:
            # Try to stop it gracefully. SIGINT is the equivilant to doing ctrl-c
            WebServer.interface_process.send_signal(signal.SIGINT)
            time.sleep(3)
            # Still running?
            if WebServer.interface_process.poll() is None:
                # Kill it with fire
                WebServer.interface_process.terminate()
                # Wait for it to finish
                WebServer.interface_process.wait()

            WebServer.interface_process = None

        # Stop referee state listener
        WebServer.timer.cancel()

        # Brake car
        WebServer.car_controls.brake = 1
        WebServer.client.setCarControls(WebServer.car_controls)
        WebServer.car_controls.brake = 0 # Remove brake

        # Create log message
        log = '{}: {}'.format(str(datetime.now()), 'Mission stopped.')
        WebServer.logs.append(log)
        del WebServer.logs[:] # Clear logs

        # Write logs to file
        WebServer.log_file.write(log + '\n')
        WebServer.log_file.close()

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
        WebServer.client.reset()
        
        log = '{}: {}'.format(str(datetime.now()), 'Car reset.')

        if WebServer.interface_process is not None:
            # Create log message and write to file
            WebServer.logs.append(log)
            WebServer.log_file.write(log + '\n')

        return {'response': log}

    @route('/logs', methods=['POST'])
    def get_logs(self):
        # Abort if access token is incorrect
        if request.json is not None and request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')  

        return {'response': WebServer.logs}

    def referee_state_listener(self):
        WebServer.timer = Timer(0.5, self.referee_state_listener)
        WebServer.timer.start()
        ref = WebServer.client.getRefereeState()

        if WebServer.doo_count != ref.doo_counter:
            delta = ref.doo_counter - WebServer.doo_count

            for d in range(WebServer.doo_count + 1, WebServer.doo_count + delta + 1):
                log = '{}: {}. {} {}'.format(str(datetime.now()), 'Cone hit', str(d), 'DOO cone(s).')
                WebServer.logs.append(log)
                WebServer.log_file.write(log + '\n')

            WebServer.doo_count = ref.doo_counter

        if len(WebServer.lap_times) != len(ref.laps):
            WebServer.lap_times = ref.laps
            lap_count = len(WebServer.lap_times)
            lap_time = WebServer.lap_times[-1]
            log = '{}: {}'.format(str(datetime.now()), 'Lap ' + str(lap_count) + ' completed. Lap time: ' + str(round(lap_time, 3)) + ' s.')
            WebServer.logs.append(log) 
            WebServer.log_file.write(log + '\n')

@app.errorhandler(400)
def bad_request(e):
    return jsonify(error=str(e)), 400

@app.errorhandler(403)
def unauthorized(e):
    return jsonify(error=str(e)), 403

WebServer.register(app, route_base='/')

if __name__ == '__main__':
    app.run()