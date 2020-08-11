#!/usr/bin/env python
from flask import Flask, request, abort, render_template, jsonify
import socket, logging, subprocess, time, signal, sys, os, errno, json, sys
from datetime import datetime
from threading import Timer
sys.path.append('..\AirSim\PythonClient')
import airsim.client as airsim


class Operator:

    def __init__(self):

        # The list of log-lines shown in the operator web gui
        self.logs = []

        # The team that is currently being simulated
        self.team = None

        # The mission that the team should use
        self.mission = None

        # The unreal engine map that is loaded when starting the simulator.
        self.track = None

        # The process descriptor of the simulator
        self.simulation_process = None

        # For every simulation launch, a new logfile is created and referee state (like lap times and DOO cones ) are stored.
        # This is the file object for the current run.
        self.log_file = None

        # Wether or not competition mode is enabled or disabled (boolean)
        self.competition_mode = None 

        # Wether or not the finished signal is received
        self.finished_signal_received = False

        # The rpc client connected to the simulation. Used to retrieve referee state.
        self.client_airsim = None
        
        # The timer that triggers periodic referee state updates.
        self.referee_state_timer = None

        # Token used to authorize requests to the operator.
        self.access_token = "1234567890"

        # Password spectators can use to connect to the simulation
        self.spectator_token = "password"

        with open('../config/team_config.json', 'r') as file:
            self.team_config = json.load(file)
            for obj in self.team_config['teams']:
                obj['car_settings']['SpectatorServerPassword'] = self.spectator_token

    def index(self):
        return render_template('index.html', teams=self.team_config['teams'], logs=self.logs)

    def launch_simulator(self):
        self.check_accesstoken()

        # Abort if simulator is already running
        if self.simulation_process is not None:
            abort(400, description='Simulation already running.')

        # Get team config
        teamId = request.json['id']
        self.mission = request.json['mission']
        self.track = request.json['track']
        self.competition_mode = request.json['competition_mode']
        for obj in self.team_config['teams']: 
            if obj['id'] == teamId: 
                self.team = obj  

        # Create log file. Create logs directory if it does not exist
        filename = 'logs/{}_{}_{}.txt'.format(str(datetime.now().strftime("%d-%m-%Y_%H%M%S")), self.team['name'].lower().replace(' ', '-'), self.mission)
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        # Write to log file
        self.log_file = open(filename, 'w')
        self.log("created logfile " + filename)

        self.finished_signal_received = False

        # Write team specific car settings to settings.json
        filename = os.path.realpath(os.path.dirname(__file__)) + '/../settings.json'
        with open(filename, 'w') as file:
            json.dump(self.team['car_settings'], file, sort_keys=True, indent=4, separators=(',', ': '))

        try:
            # Launch Unreal Engine simulator
            proc = subprocess.Popen(['../simulator/FSDS.exe', '/Game/'+self.track+'?listen'])

            time.sleep(5)

            # Create connection with airsim car client
            self.client_airsim = airsim.CarClient()
            self.client_airsim.confirmConnection()

            # Get referee state
            ref =  self.client_airsim.getRefereeState()
            self.doo_count = ref.doo_counter
            self.lap_times = ref.laps

            # Start referee state listener
            self.referee_state_timer = Timer(1.5, self.referee_state_listener)
            self.referee_state_timer.start()

            self.simulation_process = proc

            self.log('Launched simulator. {} {} {}'.format(self.team['name'], self.track, self.mission))
            return {}  
        except:
            e = sys.exc_info()[0]
            print("Error while launching simulator", e)
            self.shutdown_process(proc)
            raise

    def exit_simulator(self):
        self.check_accesstoken()

        # Abort if simulator is not running
        if self.simulation_process is None:
            abort(400, description='Simulation not running.')
    
        # Close airsim client connection
        self.client_airsim = None
        self.referee_state_timer = None
        self.team = None

        # Shutdown simulation processes
        self.shutdown_process(self.simulation_process)
        self.simulation_process = None
    
        self.log('Exited simulator.')
        
        return {}  

    def get_config(self):
        self.check_accesstoken()
        if self.team is None:
            return {}
        return {
            'team': self.team,
            'mission': self.mission,
            'track': self.track,
            'competition_mode': self.competition_mode,
        }

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

    def poll_server_state(self):
        self.check_accesstoken() 

        return {
            'logs': self.logs,
            'simulator_state': True if self.simulation_process is not None else False,
        }

    def finished(self):
        if self.finished_signal_received:
            return {}
        self.log("Received finished signal from autonomous system.")
        self.finished_signal_received = True
        return {}

    def check_accesstoken(self):
        # Abort if empty request
        if request.json is None or 'access_token' not in request.json:
            abort(400, description='Empty request.')  

        # Abort if access token is incorrect
        if request.json['access_token'] != self.access_token:
            abort(403, description='Incorrect access token')

    def referee_state_listener(self):
        if self.referee_state_timer is None:
            return
        self.referee_state_timer = Timer(1.5, self.referee_state_listener)
        self.referee_state_timer.start()
        ref = self.client_airsim.getRefereeState()

        if self.doo_count != ref.doo_counter:
            delta = ref.doo_counter - self.doo_count

            for d in range(self.doo_count + 1, self.doo_count + delta + 1):
                self.log('Cone hit. {} cone(s) DOO.'.format(d))
                
            self.doo_count = ref.doo_counter

        if len(self.lap_times) != len(ref.laps):
            self.lap_times = ref.laps
            lap_count = len(self.lap_times)
            lap_time = self.lap_times[-1]
            self.log('Lap ' + str(lap_count) + ' completed. Lap time: ' + str(round(lap_time, 3)) + ' s.')

    def log(self, line):
        line = str(datetime.now()) + ": " + line
        self.logs.append(line)
        self.log_file.write(line + '\n')
        print(line, flush=True)

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

    @app.route("/poll",  methods=['POST'])
    def poll():
        return operator.poll_server_state()

    # Get information about the current run.
    # Used by the bridge launcher to get details on how to launch the bridge.
    @app.route("/config",  methods=['POST'])
    def config():
        return operator.get_config()

    # Used by the ros bridge to let the operator know the vehicle sent a finished signal.
    @app.route("/finished",  methods=['POST'])
    def finished():
        return operator.finished()

    app.run(host= '0.0.0.0')
