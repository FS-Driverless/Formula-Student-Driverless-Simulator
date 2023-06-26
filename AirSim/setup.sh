#! /bin/bash
set -x
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

downloadHighPolySuv=true
MIN_CMAKE_VERSION=3.10.0
function version_less_than_equal_to() { test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"; }

# Parse command line arguments
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --no-full-poly-car)
    downloadHighPolySuv=false
    shift # past value
    ;;
esac
done

#install clang and build tools
VERSION=$(lsb_release -rs | cut -d. -f1)
# Since Ubuntu 17 clang is part of the core repository
# See https://packages.ubuntu.com/search?keywords=clang-8
if [ "$VERSION" -lt "22" ]; then
    wget https://apt.llvm.org/llvm.sh
    chmod +x llvm.sh
    sudo ./llvm.sh 14 all
fi

#install additional tools
sudo apt-get install -y build-essential
sudo apt-get install -y unzip

if ! which cmake; then
    # CMake not installed
    cmake_ver=0
else
    cmake_ver=$(cmake --version 2>&1 | head -n1 | cut -d ' ' -f3 | awk '{print $NF}')
fi

#download cmake - v3.10.2 is not out of box in Ubuntu 16.04
if version_less_than_equal_to $cmake_ver $MIN_CMAKE_VERSION; then
    if [[ ! -d "cmake_build/bin" ]]; then
        echo "Downloading cmake..."
        wget https://cmake.org/files/v3.10/cmake-3.10.2.tar.gz \
            -O cmake.tar.gz
        tar -xzf cmake.tar.gz
        rm cmake.tar.gz
        rm -rf ./cmake_build
        mv ./cmake-3.10.2 ./cmake_build
        pushd cmake_build
        ./bootstrap
        make
        popd
    fi
else
    echo "Already have good version of cmake: $cmake_ver"
fi

echo "Installing Eigen library..."

if [ ! -d "AirLib/deps/eigen3" ]; then
    echo "Downloading Eigen..."
    wget -O eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip
    unzip eigen3.zip -d temp_eigen
    mkdir -p AirLib/deps/eigen3
    mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3
    rm -rf temp_eigen
    rm eigen3.zip
else
    echo "Eigen is already installed."
fi

popd >/dev/null

set +x
echo ""
echo "************************************"
echo "AirSim setup completed successfully!"
echo "************************************"
