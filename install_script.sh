# Install blender dependencies
sudo ./build_environment/install_deps.sh

#
if [ -e /usr/bin/python3.6 ]
then
  echo "Using already installed python3.6 version."
else
  echo "Linking python3.6 to /opt/lib/python-3.6."
  sudo ln -s /opt/lib/python-3.6/bin/python3.6 /usr/bin/python3.6
fi

# Install script dependencies
sudo python3.6 ./scripts/setup.py install
