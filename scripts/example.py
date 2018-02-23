from framework.handler import TestHandler, BlenderHandler
from framework.objects import Cylinder, Plane
from framework.animations import Rotation, Translation

params = dict()

# image parameters
params['api_key'] = u'7f37f84f7b8454767ff11c683ba2da67'
params['api_secret'] = u'6e78737b9b53e6cb'
params['keywords'] = 'plane'
params['background_keywords'] = 'bird'
params['template_keywords'] = 'plane'
params["n_images"] = 2

# object parameters
params["object_faces"] = 200
params["object_radius"] = 0.01
params["object_length"] = 0.05
params["object_init_location"] = (0, 0, -0.15)

params["object_width"] = 0.01
params["object_height"] = 0.04
#
params["blender_output"] = "/home/ga25vug/output"

tpl_obj = Cylinder(0.01, 0.05, 200, "/home/sebastian/catkin_ws/src/tplsearch/img/template_image.jpg")
tpl_obj.set_location((-0, 0, -0.15))

# create object
# plane = Plane(params)

# init blender handler

blender_handler = BlenderHandler("/home/sebastian/output")
blender_handler.set_background("/home/sebastian/catkin_ws/src/tplsearch/img/background_image.jpg")
blender_handler.use_environment_light(True)

# add animation to plane
Rotation(blender_handler, tpl_obj, 100)

blender_handler.animation_and_save()

# init test handler
# test = TestHandler(blender_handler, params)

# test.run_tests(cyl_obj)
