# Install Python 3.6
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install python3.6
sudo apt-get install python3.6-dev

# Install setuptools
wget https://bootstrap.pypa.io/ez_setup.py -O - | sudo python3.6

# Install blender dependencies
sudo ./build_environment/install_deps.sh

# Install script dependencies
sudo python3.6 ./scripts/setup.py install
