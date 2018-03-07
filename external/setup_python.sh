sudo apt-get install python3-pip

pip3 install pillow
pip3 install flickrapi
pip3 install pyyaml

sudo touch /usr/local/lib/python3.5/dist-packages/site-packages.pth
sudo echo -e "/usr/lib/python3.5/site-packages" | sudo tee /usr/local/lib/python3.5/dist-packages/site-packages.pth
