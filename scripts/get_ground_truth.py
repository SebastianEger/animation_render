import argparse, importlib, bpy, math

from framework.handler import BlenderHandler
from framework.objects import Plane

parser = argparse.ArgumentParser(description="Process parameters.")
parser.add_argument("--anim", nargs=1)
parser.add_argument("--frames", nargs=1)
parser.add_argument("--path", nargs=1)
a = parser.parse_args()

bdr_handler = BlenderHandler("")

object = Plane(0.02, 0.02, 0)

animation_class = getattr(importlib.import_module("framework.animations"), a.anim[0])
animation_class(bdr_handler, object, int(a.frames[0]))

bdr_handler.set_interpolation(animation_class.interpolation)

posFile = open(a.path[0] + '/pos', 'w')
rotFile = open(a.path[0] + '/rot', 'w')

for frame in range(1, int(a.frames[0])+1):
    bpy.context.scene.frame_set(frame)
    # print(bpy.data.objects["Cylinder"].location)
    posFile.write(str(bpy.data.objects["Plane"].location.x) + " ")
    posFile.write(str(bpy.data.objects["Plane"].location.y) + " ")
    posFile.write(str(bpy.data.objects["Plane"].location.z) + "\n")


    ax = bpy.data.objects["Plane"].rotation_euler.x
    if(ax > math.pi):
        while(ax > math.pi):
            ax = ax - 2*math.pi
    elif(ax < -math.pi):
        while(ax < -math.pi):
            ax = ax + 2*math.pi

    ay = bpy.data.objects["Plane"].rotation_euler.y
    if(ay > math.pi):
        while(ay > math.pi):
            ay = ay - 2*math.pi
    elif(ay < -math.pi):
        while(ay < -math.pi):
            ay = ay + 2*math.pi

    az = bpy.data.objects["Plane"].rotation_euler.z
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
