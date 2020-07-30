#!/usr/bin/env python
from flask import Flask, request, abort, render_template, jsonify
import logging
import socket
from datetime import datetime
from threading import Timer
import subprocess, time, signal, sys, os, errno, json, sys
sys.path.append('..\AirSim\PythonClient')
import airsim.client as airsim


class Operator:

    def __init__(self):
        self.simulation_process = None
        self.interface_process =  None
        self.log_file = None
        self.logs = []

        self.team = None
        self.mission = None
        self.client = None
        self.car_controls = None
        self.referee_state_timer = None

        with open('../config/team_config.json', 'r') as file:
            self.team_config = json.load(file)
            self.access_token = self.team_config['access_token']

    def index(self):
        return render_template('index.html', teams=self.team_config['teams'], logs=self.logs)

    def launch_simulator(self):
        # Abort if empty request
        if request.json is None or not all(key in request.json for key in ['id', 'mission', 'access_token']):
            abort(400, description='Empty request.')   

        # Abort if access token is incorrect
        if request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if simulator is already running
        if self.simulation_process is not None:
            abort(400, description='Simulation already running.')

        print("Creating logfile for run")

        # Get team config
        teamId = request.json['id']
        self.mission = request.json['mission']
        track = request.json['track']
        for obj in self.team_config['teams']: 
            if obj['id'] == teamId: self.team = obj  

        # Create log file. Create logs directory if it does not exist
        filename = 'logs/{}_{}_{}.txt'.format(self.team['name'].lower().replace(' ', '-'), self.mission, str(datetime.now().strftime("%d-%m-%Y_%Hh%Mm%Ss")))
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        # Write to log file
        self.log_file = open(filename, 'w')
        self.log_file.write('Launching simulator '+track+'...\n')

        print("Launching simulator...")
       

        # Write team specific car settings to settings.json
        filename = os.path.realpath(os.path.dirname(__file__)) + '/../settings.json'
        with open(filename, 'w') as file:
            json.dump(self.team['car_settings'], file, sort_keys=True, indent=4, separators=(',', ': '))

        try:
            # Launch Unreal Engine simulator
            proc = subprocess.Popen(['../simulator/FSDS.exe', '/Game/'+track+'?listen'])

            time.sleep(7)

            # Create connection with airsim car client
            self.client = airsim.CarClient()
            self.client.confirmConnection()

            # Get referee state
            ref =  self.client.getRefereeState()
            self.doo_count = ref.doo_counter
            self.lap_times = ref.laps 

            self.simulation_process = proc
            print("Launching simulator done")

            log = '{}: {}'.format(str(datetime.now()), 'Launched simulator. Team: ' + self.team['name'] + '.')
            self.logs.append(log)
            return {'response': log}  
        except:
            e = sys.exc_info()[0]
            print("Error while launching simulator", e)
            self.shutdown_process(proc)
            raise

    def exit_simulator(self):
        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')  

        # Abort if access token is incorrect
        if request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if simulator is not running
        if self.simulation_process is None:
            abort(400, description='Simulation not running.')

        if self.interface_process is not None:
            abort(400, description='Mission still active.')
    
        # Close airsim client connection
        print("Closing clients")
        self.client = None
        self.referee_state_timer = None

        # Shutdown simulation processes
        self.shutdown_process(self.simulation_process)
        self.simulation_process = None
    
        log = '{}: {}'.format(str(datetime.now()), 'Exited simulator.')
        self.logs.append(log)
        
        return {'response': log}  

    def shutdown_process(self, proc):
        if proc is None:
            return
        if proc.poll() is None:
            # process has not (yet) terminated. 
            
            # Try to stop it gracefully.
            os.kill(proc.pid, signal.CTRL_BREAK_EVENT)
            time.sleep(3)

            # Going to kill all related processes created by simulation_process
            os.system("taskkill /im Blocks* /F")
            
            # Still running?
            if proc.poll() is None:
                # Kill it with fire
                proc.terminate()
                # Wait for it to finish
                proc.wait()

    
    def mission_start(self):
        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')   

        # Abort if access token is incorrect
        if request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token') 

        # Abort if Unreal Engine simulator is not running
        if self.simulation_process is None:
            abort(400, description='Simulator not running.') 

        if self.interface_process is not None:
            abort(400, description='Mission already running.')

        # Launch ROS bridge
        localip = socket.gethostbyname(socket.gethostname())
        print('ROS_IP = ' + localip)
        print('ROS_MASTER_URI=' + self.team['master'])
        self.interface_process = subprocess.Popen('ubuntu1804 run source /opt/ros/melodic/setup.bash; export ROS_IP='+localip+'; export ROS_MASTER_URI='+self.team['master']+'; source ~/Formula-Student-Driverless-Simulator/ros/devel/setup.bash; roslaunch fsds_ros_bridge fsds_ros_bridge.launch competition_mode:=true mission_name:='+self.mission+' access_token:='+self.access_token, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)

        # Start referee state listener
        self.referee_state_listener()

        log = '{}: {}'.format(str(datetime.now()), 'Mission ' + self.mission + ' started.')
        self.logs.append(log)

        return {'response': log}

    def mission_stop(self):
        # Abort if empty request
        if request.json is None or not all(key in request.json for key in ['sender', 'access_token']):
            abort(400, description='Empty request.')   

        # Abort if access token is incorrect
        if request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if Unreal Engine simulator is not running
        if self.simulation_process is None:
            abort(400, description='Simulator not running.') 

        # Abort if ROS bridge is not running
        if self.interface_process is None:
            abort(400, description='No process running.')

        # Check if previous process is still running
        if self.interface_process.poll() is None:
            # Try to stop it gracefully.
            os.kill(self.interface_process.pid, signal.CTRL_BREAK_EVENT)
            time.sleep(3)
            # Still running?
            if self.interface_process.poll() is None:
                # Kill it with fire
                self.interface_process.terminate()
                # Wait for it to finish
                self.interface_process.wait()

            self.interface_process = None

        # Stop referee state listener
        self.referee_state_timer.cancel()

        # Brake car
        car_controls = airsim.CarControls()
        car_controls.brake = 1
        self.client.setCarControls(car_controls)

        # Create log message
        log = '{}: {}'.format(str(datetime.now()), 'Mission ' + self.mission + ' stopped by ' + request.json['sender'] + '.')
        self.logs.append(log)
        if request.json['sender'] == 'AS': time.sleep(5) # Wait 5 seconds for web interface to poll server
        del self.logs[:] # Clear logs

        # Write logs to file
        self.log_file.write(log + '\n')
        self.log_file.close()
        self.log_file = None

        return {'response': log}

    def poll_server_state(self):
        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.') 

        # Abort if access token is incorrect
        if request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token') 

        return {
            'logs': self.logs,
            'simulator_state': True if self.simulation_process is not None else False,
            'interface_state': True if self.interface_process is not None else False
        }

    def referee_state_listener(self):
        self.referee_state_timer = Timer(1.5, self.referee_state_listener)
        self.referee_state_timer.start()
        ref = self.client.getRefereeState()

        if self.doo_count != ref.doo_counter:
            delta = ref.doo_counter - self.doo_count

            for d in range(self.doo_count + 1, self.doo_count + delta + 1):
                log = '{}: {}. {} {}'.format(str(datetime.now()), 'Cone hit', str(d), 'cone(s) DOO.')
                self.logs.append(log)
                self.log_file.write(log + '\n')

            self.doo_count = ref.doo_counter

        if len(self.lap_times) != len(ref.laps):
            self.lap_times = ref.laps
            lap_count = len(self.lap_times)
            lap_time = self.lap_times[-1]
            log = '{}: {}'.format(str(datetime.now()), 'Lap ' + str(lap_count) + ' completed. Lap time: ' + str(round(lap_time, 3)) + ' s.')
            self.logs.append(log) 
            self.log_file.write(log + '\n')


if __name__ == '__main__':
    operator = Operator()

    app = Flask(__name__)
    werkzeuglogs = logging.getLogger('werkzeug')
    werkzeuglogs.setLevel(logging.ERROR)


    @app.errorhandler(400)
    def bad_request(e):
        return jsonify(error=str(e)), 400

    @app.errorhandler(403)
    def unauthorized(e):
        return jsonify(error=str(e)), 403

    @app.errorhandler(500)
    def unauthorized(e):
        return jsonify(error=str(e)), 500

    @app.route("/")
    def index():
        return operator.index()

    @app.route("/simulator/launch", methods=['POST'])
    def launch_simulator():
        return operator.launch_simulator()

    @app.route("/simulator/exit",  methods=['POST'])
    def exit():
        return operator.exit_simulator()

    @app.route("/mission/start",  methods=['POST'])
    def mission_start():
        return operator.mission_start()

    @app.route("/mission/stop",  methods=['POST'])
    def mission_stop():
        return operator.mission_stop()

    @app.route("/poll",  methods=['POST'])
    def poll():
        return operator.poll_server_state()

    app.run(host= '0.0.0.0')
