import argparse
import math

from framework.handler import BlenderHandler
from framework import objects

parser = argparse.ArgumentParser(description="Process parameters.")
parser.add_argument("--path", nargs=1)
parser.add_argument("--res",  nargs=2)
parser.add_argument("--w",    nargs=1)
parser.add_argument("--h",    nargs=1)
parser.add_argument("--img",  nargs=1)
a = parser.parse_args()

bldr_handler = BlenderHandler(None)
bldr_handler.use_environment_light(True)
bldr_handler.set_resolution(int(a.res[0]), int(a.res[1]))
bldr_handler.set_horizon_color( (1, 1, 1) )
bldr_handler.use_anti_aliasing(False)

cylinder = objects.Plane(a.w[0], a.h[0], 0, a.img[0])

cylinder.set_location((0, 0, -0.10))

file_name = "/home/sebastian/catkin_ws/src/tplsearch/img/plane/template.png"

bldr_handler.render_and_save(file_name)
