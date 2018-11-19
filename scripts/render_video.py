import argparse, math, random, bpy
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
if( a.bgr_img[0] == "White.jpg"):
    bldr_handler.set_horizon_color((1.0, 1.0, 1.0))
else:
    bldr_handler.set_background(a.bgr_img[0])
bldr_handler.set_fps(int(a.fps[0]))
bldr_handler.use_environment_light(True)
bldr_handler.set_environment_energy(1)

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
animation = animation_class(bldr_handler, template_object, int(a.frames[0]))

# Set interpolation
bldr_handler.set_interpolation(animation.interpolation)

# Noise
action = template_object.obj.animation_data.action

if False:
    for fcu in action.fcurves:
        if fcu.data_path == "location":
            mod = fcu.modifiers.new("NOISE")
            mod.strength = 0.05
        if fcu.data_path == "rotation_euler":
            mod = fcu.modifiers.new("NOISE")
            mod.strength = 0.5
# Render
bldr_handler.animation_and_save()

# Ground Truth
filename = a.out[0]
filename = filename[:-4]
posFile = open(filename + '_pos', 'w')
rotFile = open(filename + '_rot', 'w')

for frame in range(1, int(a.frames[0])+1):
    bldr_handler.scene.frame_set(frame)
    # print(bpy.data.objects["Cylinder"].location)
    posFile.write(str(template_object.obj.location.x) + " ")
    posFile.write(str(template_object.obj.location.y) + " ")
    posFile.write(str(template_object.obj.location.z) + "\n")


    ax = template_object.obj.rotation_euler.x
    if(ax > math.pi):
        while(ax > math.pi):
            ax = ax - 2*math.pi
    elif(ax < -math.pi):
        while(ax < -math.pi):
            ax = ax + 2*math.pi

    ay = template_object.obj.rotation_euler.y
    if(ay > math.pi):
        while(ay > math.pi):
            ay = ay - 2*math.pi
    elif(ay < -math.pi):
        while(ay < -math.pi):
            ay = ay + 2*math.pi

    az = template_object.obj.rotation_euler.z
    if(az > math.pi):
        while(az > math.pi):
            az = az - 2*math.pi
    elif(az < -math.pi):
        while(az < -math.pi):
            az = az + 2*math.pi

    # rotFile.write(str(bpy.data.objects["Plane"].rotation_euler.x) + " ")
    rotFile.write(str(ax) + " ")
    rotFile.write(str(ay) + " ")
    rotFile.write(str(az) + "\n")
