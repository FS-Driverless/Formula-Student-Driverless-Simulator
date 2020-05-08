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

Do not enable NVIDIA GRID

Choose Ubuntu 18.04 LTS

Ensure this instance has a public ip.

## 2. Install NVIDIA drivers
Install the NVIDIA drivers. Based on [this tutorial from gcp](https://cloud.google.com/compute/docs/gpus/install-drivers-gpu#ubuntu-driver-steps).
```
curl -O http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sudo apt-get update
sudo apt-get install -y cuda vulkan-utils
```
Verify validation by running `nvidia-smi`:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 440.64.00    Driver Version: 440.64.00    CUDA Version: 10.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  Tesla P100-PCIE...  On   | 00000000:00:04.0 Off |                    0 |
| N/A   38C    P0    26W / 250W |      0MiB / 16280MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
```
and `vulkaninfo`:
```
===========
VULKAN INFO
===========

Vulkan Instance Version: 1.1.70

...
```


## 2. Install virtual desktop
Note: We can't _just use any remote desktop_ because we require hardware accelerated graphics rendering through vulkan.
Therefore we run a virtual X display with NVIDIA driver and share that display through vnc.  
 
Install a desktop environment:
```
sudo apt-get install -y --no-install-recommends lxde lxdm
```

Start and enable the display manager:
```
sudo systemctl start lxdm
sudo systemctl enable lxdm
```

Generate the NVIDIA X configuration file:
```
sudo nvidia-xconfig
```

edit the generated `/etc/X11/xorg.conf` and add a `Modes` to the `Screen` `SubSection`. This part should look like:
```
Section "Screen"
    Identifier     "Screen0"
    Device         "Device0"
    Monitor        "Monitor0"
    DefaultDepth    24
    SubSection     "Display"
        Depth       24 
        Modes      "1600x1024"
    EndSubSection
EndSection

```
Note that the resolution can be higher. To see the effect of changes to this file, run `sudo systemctl restart lxdm`

Install the vnc server:

```
sudo apt-get install -y tigervnc-scraping-server
```

Now we are going to create a service that will launch the vnc server.

Create the file `/etc/systemd/system/vncdisplay.service` with this content:
```
[Unit]
Description=Startx
After=xdisplay.service

[Service]
Environment=XAUTHORITY=/var/run/lxdm/lxdm-:0.auth
ExecStartPre=/bin/sleep 30
ExecStart=/usr/bin/x0tigervncserver -SecurityTypes=none -display :0
Restart=on-abort

[Install]
WantedBy=graphical.target
```

Load, start and enable the service:
```
sudo systemctl daemon-reload
sudo systemctl start vncdisplay
sudo systemctl enable vncdisplay
```

Check if everything is working ok run 
```
sudo systemctl status vncdisplay
```
output should be like this:
```
● vncdisplay.service - Startx
   Loaded: loaded (/etc/systemd/system/vncdisplay.service; enabled; vendor preset: enabled)
   Active: active (running) since Tue 2020-04-28 13:44:04 UTC; 6s ago
 Main PID: 20234 (x0tigervncserve)
    Tasks: 1 (limit: 4915)
   CGroup: /system.slice/vncdisplay.service
           └─20234 /usr/bin/x0tigervncserver -SecurityTypes=none -display :0
```

## 3. Configure your local computer to view the remote desktop

Open an ssh tunnel towards the gcp instance:
```
ssh -NL 5900:localhost:5900 <user>@<ip>
```
Replace `<user>` with your ssh username and `<ip>` with the ip of the remote device.
This will forward local port 5900 to the instance port 5900 through an secure ssh connection.
You have this keep this command running while working on the remote desktop.  
 
To view the remote desktop, install vncviewer:
```
sudo apt-get install -y tigervnc-viewer
```

Finally open the vnc connection and work on the desktop:
```
vncviewer DotWhenNoCursor=1 localhost:5900
```
to enter/leave full screen press the `F8` button. 

You now should see a login screen.
To be able to login you must set a password for the account:
```
sudo passwd <user>
``` 
