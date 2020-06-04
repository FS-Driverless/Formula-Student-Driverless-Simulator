# Operator

The operator consists of both a web interface and a webserver. 
The operator is meant to be used by Formula Student officials to control and keep track of what is happening in the simulation.
From this web interface, the official can launch and exit the simulator, select teams and events, start, stop and reset the car and view all ogs received by the webserver.
All these logs are also stored in log files, in the case that the operator crashes.

**The operator is primarily used during competition by officials. You don't need this for development and testing.**
Refer to the [how-to-simulate](how-to-simulate.md) and [how-to-develop](how-to-develop.md) guide first.

![Operator](images/operator.png)

## Prerequisites

The operator only works on windows with wsl.
The operator and unreal engine simulator will run in Windows, the ros bridge will run in wsl.
Before we start you must build the ros workspace in wsl and clone the repo in windows.

+ [Flask](https://flask.palletsprojects.com/en/1.1.x/) - A Python web application framework.
+ [Flask-Classful](http://flask-classful.teracy.org/) - An extension that adds class based views to Flask

To install all dependencies, run the following command inside the `operator` folder:
```bash
$ pip install -r requirements.txt
```

You must have a packaged simulator downloaded.
The operator will launch the game when instructed by the user via the web gui.
Go to the [releases](https://github.com/FS-Online/Driverless-Competition-Simulator/releases) and download the latest version.
Extract the zip to the `simulator` folder.
The result should be that the following file and folders exist inside the `simulator` folder:
* FSDS.exe
* FSOnline/
* Engine/
The [how-to-develop guide](how-to-develop.md) guide describes how to create an export.

## Usage
To start the web server, run the following command in the `operator` folder:
```bash
$ python webserver.py
```
By default, the web interface runs on `http://localhost:5000`.
