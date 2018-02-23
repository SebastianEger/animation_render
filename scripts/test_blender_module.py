import os, sys

pathname = os.path.realpath(__file__)
pathname = pathname[:-30] + "lib/"
sys.path.append(pathname)
import bpy
