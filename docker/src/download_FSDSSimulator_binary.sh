# Shell script to download Simulator bins from github; For custom versions of simulator path has to be updated
sudo apt-get install unzip
wget -c https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.0.0/fsds-v2.0.0-linux.zip
unzip fsds-v2.0.0-linux.zip
rm fsds-v2.0.0-linux.zip