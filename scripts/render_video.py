import argparse
from framework.handler import BlenderHandler
import importlib

parser = argparse.ArgumentParser(description="Process parameters.")
parser.add_argument("--frames", nargs=1)
parser.add_argument("--obj", nargs=1)
parser.add_argument("--mp", nargs=3)
parser.add_argument("--tpl_img", nargs=1)
parser.add_argument("--bgr_img", nargs=1)
parser.add_argument("--out", nargs=1)
parser.add_argument("--anim", nargs=1)
parser.add_argument("--res", nargs=2)
parser.add_argument("--fps", nargs=1)
parser.add_argument("--focal", nargs=1)
parser.add_argument("--sensor", nargs=1)
a = parser.parse_args()

# bpy.context.scene.cycles.device = 'GPU'

bldr_handler = BlenderHandler(a.out[0])
bldr_handler.set_background(a.bgr_img[0])
bldr_handler.set_fps(int(a.fps[0]))
bldr_handler.use_environment_light(True)

if(a.focal[0] is not None):
    bldr_handler.set_focal_length(float(a.focal[0]))
else:
    bldr_handler.set_focal_length(35.0)
if(a.sensor[0] is not None):
    bldr_handler.set_sensor_width(float(a.sensor[0]))
else:
    bldr_handler.set_sensor_width(32.0)

if int(a.res[0]) > 0 and int(a.res[1]) > 0:
    bldr_handler.set_resolution(int(a.res[0]), int(a.res[1]))

# Create object
template_class = getattr(importlib.import_module("framework.objects"), a.obj[0])
template_object = template_class(a.mp[0], a.mp[1], a.mp[2], image=a.tpl_img[0])

# Add animation
animation_class = getattr(importlib.import_module("framework.animations"), a.anim[0])
animation_class(bldr_handler, template_object, int(a.frames[0]))

# Render
bldr_handler.animation_and_save()
