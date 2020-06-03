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

    # Class variables are used for variables that can change while the web server is running
    # This has to do with how flask_classful handles instance variables
    simulation_process = None
    interface_process =  None
    log_file = None
    logs = []

    team = None
    mission = None
    client = None
    car_controls = None
    referee_state_timer = None

    def __init__(self):
        # Instance variables are used for variables that don't change while the web server is running
        with open('../config/team_config.json', 'r') as file:
            self.team_config = json.load(file)
            self.access_token = self.team_config['access_token']

    @route('/', methods=['GET'])
    def index(self):
        return render_template('index.html', teams=self.team_config['teams'], logs=WebServer.logs)

    @route('/simulator/launch', methods=['POST'])
    def launch_simulator(self):
        # Abort if access token is incorrect
        if request.json is not None and request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if empty request
        if request.json is None or not all(key in request.json for key in ['id', 'mission', 'access_token']):
            abort(400, description='Empty request.')   

        # Abort if simulator is already running
        if WebServer.simulation_process is not None:
            abort(400, description='Simulation already running.')

        # Get team config
        teamId = request.json['id']
        WebServer.mission = request.json['mission']
        for obj in self.team_config['teams']: 
            if obj['id'] == teamId: WebServer.team = obj  

        # Write team specific car settings to settings.json
        filename = os.path.realpath(os.path.dirname(__file__)) + '/../UE4Project/Plugins/AirSim/Settings/settings.json'
        with open(filename, 'w') as file:
            json.dump(WebServer.team['car_settings'], file, sort_keys=True, indent=4, separators=(',', ': '))

        # Launch Unreal Engine simulator
        WebServer.simulation_process = subprocess.Popen(['./../UE4Project/LinuxNoEditor/Blocks.sh'])  
        time.sleep(7)

        # Create connection with airsim car client
        WebServer.client = airsim.CarClient()
        WebServer.client.confirmConnection()
        WebServer.car_controls = airsim.CarControls()

        # Get referee state
        ref =  WebServer.client.getRefereeState()
        WebServer.doo_count = ref.doo_counter
        WebServer.lap_times = ref.laps 

        log = '{}: {}'.format(str(datetime.now()), 'Launched simulator. Team: ' + WebServer.team['name'] + '.')
        return {'response': log}  

    @route('/simulator/exit', methods=['POST'])
    def exit_simulator(self):
        # Abort if access token is incorrect
        if request.json is not None and request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')   

        # Abort if simulator is not running
        if WebServer.simulation_process is None:
            abort(400, description='Simulation not running.')

        if WebServer.interface_process is not None:
            abort(400, description='Mission still active.')
    
        # Close airsim client connection
        WebServer.client = None
        WebServer.car_controls = None
        WebServer.referee_state_timer = None

        # Shutdown simulation processes
        if WebServer.simulation_process.poll() is None:
            # Kill process created by simulation_process
            child_process_id = int(subprocess.check_output(['pidof', 'Blocks']))
            os.kill(child_process_id, signal.SIGINT)

            # Check if procses has been killed, if so pass, else terminate
            try:
                os.kill(child_process_id, 0)
            except OSError:
                pass
            else:
                os.kill(child_process_id, signal.SIGTERM)

            # Try to stop it gracefully. SIGINT is the equivilant to doing ctrl-c
            WebServer.simulation_process.send_signal(signal.SIGINT)
            time.sleep(3)
            # Still running?
            if WebServer.simulation_process.poll() is None:
                # Kill it with fire
                WebServer.simulation_process.terminate()
                # Wait for it to finish
                WebServer.simulation_process.wait()
                #print WebServer.simulation_process

            WebServer.simulation_process = None
    
        log = '{}: {}'.format(str(datetime.now()), 'Exited simulator.')
        return {'response': log}  

    @route('/mission/start', methods=['POST'])
    def mission_start(self):
        # Abort if access token is incorrect
        if request.json is not None and request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')   

        # Abort if Unreal Engine simulator is not running
        if WebServer.simulation_process is None:
            abort(400, description='Simulator not running.') 

        if WebServer.interface_process is not None:
            abort(400, description='Mission already running.')

        # Set ROS MASTER
        procenv = os.environ.copy()
        procenv["ROS_MASTER_URI"] = WebServer.team['master']

        # Launch ROS bridge
        WebServer.interface_process = subprocess.Popen(['roslaunch', 'fsds_ros_bridge', 'fsds_ros_bridge.launch', 'mission:={}'.format(WebServer.mission)], env=procenv)  

        # Start referee state listener
        self.referee_state_listener() 

        # Create log message
        log = '{}: {}'.format(str(datetime.now()), 'Mission ' + WebServer.mission + ' started.')
        WebServer.logs.append(log)

        # Create log file. Create logs directory if it does not exist
        filename = 'logs/{}_{}_{}.txt'.format(WebServer.team['name'].lower().replace(' ', '-'), WebServer.mission, str(datetime.now().strftime("%d-%m-%Y_%H:%M:%S")))
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
        if request.json is None or not all(key in request.json for key in ['sender', 'access_token']):
            abort(400, description='Empty request.')   

        # Abort if Unreal Engine simulator is not running
        if WebServer.simulation_process is None:
            abort(400, description='Simulator not running.') 

        # Abort if ROS bridge is not running
        if WebServer.interface_process is None:
            abort(400, description='No process running.')

        # Check if previous process is still running
        if WebServer.interface_process.poll() is None:
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
        WebServer.referee_state_timer.cancel()

        # Brake car
        WebServer.car_controls.brake = 1
        WebServer.client.setCarControls(WebServer.car_controls)
        WebServer.car_controls.brake = 0 # Remove brake

        # Create log message
        log = '{}: {}'.format(str(datetime.now()), 'Mission ' + WebServer.mission + ' stopped by ' + request.json['sender'] + '.')
        WebServer.logs.append(log)
        if request.json['sender'] == 'AS': time.sleep(5) # Wait 5 seconds for web interface to poll server
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

        if WebServer.client is None:
            abort(400, description='No connection to the AirSim client.')

        # Reset simulator
        WebServer.client.reset()
        
        log = '{}: {}'.format(str(datetime.now()), 'Car reset.')

        if WebServer.interface_process is not None:
            # Create log message and write to file
            WebServer.logs.append(log)
            WebServer.log_file.write(log + '\n')

        return {'response': log}

    @route('/poll', methods=['POST'])
    def poll_server_state(self):
        # Abort if access token is incorrect
        if request.json is not None and request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')  

        return {
            'logs': WebServer.logs,
            'simulator_state': True if WebServer.simulation_process is not None else False,
            'interface_state': True if WebServer.interface_process is not None else False
        }

    def referee_state_listener(self):
        WebServer.referee_state_timer = Timer(0.5, self.referee_state_listener)
        WebServer.referee_state_timer.start()
        ref = WebServer.client.getRefereeState()

        if WebServer.doo_count != ref.doo_counter:
            delta = ref.doo_counter - WebServer.doo_count

            for d in range(WebServer.doo_count + 1, WebServer.doo_count + delta + 1):
                log = '{}: {}. {} {}'.format(str(datetime.now()), 'Cone hit', str(d), 'cone(s) DOO.')
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