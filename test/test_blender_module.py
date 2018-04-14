import os, sys

pathname = os.path.realpath(__file__)
pathname = pathname[:-28] + "lib/"

# add blender lib to path
sys.path.append(pathname)

import bpy
