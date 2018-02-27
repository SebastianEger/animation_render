# Install Python 3.6
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install python3.6
sudo apt-get install python3.6-dev

# Install setuptools
sudo apt-get install python3-pip
sudo python3.6 -m pip install Pillow
sudo python3.6 -m pip install flickrapi
sudo python3.6 -m pip install pyyaml

# Install blender dependencies
sudo ./build_environment/install_deps.sh

# Install script dependencies
# sudo python3.6 ./scripts/setup.py install
