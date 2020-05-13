#!/usr/bin/env python

from flask import Flask, request, abort
import subprocess, time, signal, sys, os

app = Flask(__name__)

@app.route('/mission/start')
def mission_start():
    return {"message": "hello world"}

@app.route('/mission/stop')
def mission_stop():
    return 

global interfaceprocess
interfaceprocess = None

# curl --header "Content-Type: application/json" --request POST --data '{"master": "http://localhost:11311"}' http://localhost:5000/mission/selectcar

@app.route('/mission/selectcar', methods=['POST'])
def mission_changecar():
    if request.json is None or request.json['master'] is None:
        return abort(400)

    global interfaceprocess

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

    procenv = os.environ.copy()
    procenv["ROS_MASTER_URI"] = request.json['master']

    interfaceprocess = subprocess.Popen(['roslaunch', 'airsim_ros_interface', 'airsim_node.launch'], env=procenv)    

    return {'message': 'ok'}

if __name__ == '__main__':
    app.run()