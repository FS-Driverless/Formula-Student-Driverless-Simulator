# How to deploy a remote workstation on Google Cloud Platform

In this tutorial we create an gcp instance that is configured for running and developing the Driverless-Competition-Simulator. 

## 1. Create a new instance and configure ports

We assume you are familiar with google cloud configurations.
Create a new instance in the region of your choice.

Minimum requirements are:
* 12 vCPU
* 24GB memory
* Any NVIDIA GPU
* 150GB Disk

Recommended specs are:
* 16 vCPU
* 32GB memory
* 1 NVIDIA Tesla P100
* 300GB SSD Disk
* CPU platform Sandy Bridge

Do not enable NVIDIA GRID

Choose Windows Server 2019 Datacenter

Check 'Attach display device' and 'enable NVidia Grid'

Ensure this instance has a public ip.

## 2. Access the remote desktop

Go to the cloud instance and use the 'Set Windows password' to set a password for your user.

### From Ubuntu

Install a remote desktop client:
```
sudo apt-get install remmina
```

Launch Remmina, add a new connection. Set
* Server to the ip of the instance
* username to the name you entered when setting the password
* password to the password you created before
* Color depth to `High color (16 bpp)

### From Windows
Click the little arrow next to the 'RDP' button and click 'Download the RDP file'. 
Double click on the downloaded file. 
Use the credentials you just created to login.

## 3. Disable internet security
We need to disable internet security protection because we want to download a bunch of tools.

Start the Server Manager.

Select Local Server (The server you are currently on and the one that needs IE Enhanced Security disabled)

On the right side of the Server Manager, find the IE Enhanced Security Configuration Setting. Disable it.

Open Internet Options, go to tab 'Security' set the security level for 'internet' to 'Medium'. 
Disable 'Protection Mode'.

Now you can use internet explorer to downlaod firefox.

## 4. Install NVIDIA drivers

Follow [this tutorial](https://cloud.google.com/compute/docs/gpus/install-grid-drivers) to install the required nvidia drivers.

Restart your computer.

Validate the installation by running the following command in a powershell terminal
```
& 'C:\Program Files\NVIDIA Corporation\NVSMI\nvidia-smi.exe'
```

## 5. Install .NET and Windows Subsystem for Linux
Start the Server Manager.
1. Click 'Manage', 'Add Roles and Features'
2. Click Next until you find 'Server Roles'.
3. In 'Server Roles', select 'Remote Desktop Services'
4. In 'Features', select '.NET Framework 3.5' and 'Windows Subsystem for Linux'.
5. Click install. You can ignore the warning about missing source files.

Restart the computer