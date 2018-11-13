import argparse
import math

from framework.handler import BlenderHandler
from framework import objects

parser = argparse.ArgumentParser(description="Process parameters.")
parser.add_argument("--path", nargs=1)
parser.add_argument("--res",  nargs=2)
parser.add_argument("--l",    nargs=1)
parser.add_argument("--r",    nargs=1)
parser.add_argument("--f",    nargs=1)
parser.add_argument("--n",    nargs=1)
parser.add_argument("--img",  nargs=1)
a = parser.parse_args()

bldr_handler = BlenderHandler(None)
bldr_handler.use_environment_light(True)
bldr_handler.set_resolution(int(a.res[0]), int(a.res[1]))
bldr_handler.set_horizon_color( (1, 1, 1) )
bldr_handler.use_anti_aliasing(False)

cylinder = objects.Cylinder(a.l[0], a.r[0], a.f[0], a.img[0])

cylinder.set_location((0, 0, -0.15))

for i in range(int(a.n[0])):
    angle_x = i*2*math.pi/float(a.n[0])
    file_name = "/home/sebastian/catkin_ws/src/tplsearch/img/cylinder/template_" + str(i) + ".png"
    cylinder.set_rotation((angle_x, 0, 0))
    bldr_handler.render_and_save(file_name)
