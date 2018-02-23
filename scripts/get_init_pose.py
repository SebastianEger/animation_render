
import argparse, importlib

parser = argparse.ArgumentParser(description="Process parameters.")
parser.add_argument("--anim", nargs=1)
a = parser.parse_args()

animation_class = getattr(importlib.import_module("framework.animations"), a.anim[0])

pos = animation_class.get_start_position()
rot = animation_class.get_start_rotation()

print(pos[0])
print(pos[1])
print(pos[2])
print(rot[0])
print(rot[1])
print(rot[2])

# rospy.set_param("/tpltester/euler_x", rot[0])
# rospy.set_param("/tpltester/euler_y", rot[1])
# rospy.set_param("/tpltester/euler_z", rot[2])
