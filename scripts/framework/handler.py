import bpy
import flickrapi
import math
from PIL import Image

"""@package Handler
Contains various handler for blender, testing and image retrieving.
"""
class BlenderHandler:
    # parameters
    file_path = None

    # world
    world = None

    # scene
    scene = None
    camera = None

    # background
    bgr_img = None
    bgr_tex = None
    bgr_tex_slot = None

    def __init__(self, file_path):
        """!@brief Creates a new BlenderHandler

        This class contains all important functions and objects of blender.

        @param file_path Output path for rendered images and videos
        """
        self.file_path = file_path

        self._init_world()
        self._init_scene()

        bpy.data.cameras['Camera'].clip_start = 0.001

    @staticmethod
    def _remove_init_cube():
        bpy.ops.object.select_all(action='DESELECT')
        bpy.data.objects['Cube'].select = True
        bpy.ops.object.delete()

    def _init_world(self):
        self.world = bpy.data.worlds['World']
        self.world.use_sky_paper = True
        self.bgr_tex = bpy.data.textures.new("background", 'IMAGE')
        self.bgr_tex_slot = self.world.texture_slots.add()
        self.bgr_tex_slot.texture = self.bgr_tex
        self.bgr_tex_slot.use_map_horizon = True

    def _init_scene(self):
        self.scene = bpy.data.scenes['Scene']
        self.scene.render.resolution_percentage = 100
        self.camera = bpy.data.objects['Camera']
        self.camera.location = (0, 0, 0)
        self.camera.rotation_euler = (math.pi, 0, 0)
        self._remove_init_cube()

    def set_background_pil(self, image):
        """!@brief Set background image
            @param image PIL image
        """
        if not os.path.exists(".tmp"):
            os.makedirs(".tmp")

        image.save(".tmp/background.png")
        self.bgr_tex.image = bpy.data.images.load(".tmp/background.png")

        image_size = self.bgr_tex.image.size
        self.set_resolution(image_size[0], image_size[1])

    def set_background(self, image):
        """!@brief Set background image
            @param image Image path
        """
        self.bgr_tex.image = bpy.data.images.load(image)
        image_size = self.bgr_tex.image.size
        self.set_resolution(image_size[0], image_size[1])

    def set_horizon_color(self, color):
        """!@brief Set horizon color
            @param color Color in RGB
        """
        self.world.horizon_color = color

    def use_anti_aliasing(self, b):
        """!@brief Enable or disable anti aliasing
            @param b True: Enable, False: Disable
        """
        self.scene.render.use_bake_antialiasing = b
        self.scene.render.use_antialiasing = b

    @staticmethod
    def set_resolution(x, y):
        """!@brief Set resolution of rendered image or video
            @param x width in pixel
            @param y height in pixel
        """
        bpy.context.scene.render.resolution_x = x
        bpy.context.scene.render.resolution_y = y
        # print("Set render resolution to " + str(x) + "x" + str(y))

    def set_fps(self, fps):
        """!@brief Set frame per second of rendered video
            @param fps Frames per second
        """
        self.scene.render.fps = fps

    def set_focal_length(self, focal_length):
        """!@brief Set focal length of camera
            @param focal_length Focal length in mm
        """
        bpy.data.cameras['Camera'].lens = focal_length

    def set_sensor_width(self, sensor_width):
        """!@brief Set sensor width of camera
            @param sensor_width Sensor width in mm
        """
        bpy.data.cameras['Camera'].sensor_width = sensor_width

    def set_interpolation(self, interpol):
        """!@brief Set interpolation modus between keyframes. Can be set in the Animation
            @param interpol Interpolation modus. For example: "LINEAR", "BEZIER", "QUAD"
        """
        actions = bpy.data.actions
        for action in actions:
            fcurves = action.fcurves
            for fcurve in fcurves:
                for kf in fcurve.keyframe_points:
                    kf.interpolation = interpol

    def use_environment_light(self, b):
        """!@brief Enable or disable environmental lighting
            @param b True: Enable, False: Disable
        """
        self.world.light_settings.use_environment_light = b

    def set_environment_energy(self, e):
        """!@brief Set enegery of environmental lighting. Only necessary if enabled.
            @param e Energy
        """
        self.world.light_settings.environment_energy = e

    def render_and_save(self, path=None):
        """!@brief Render current image and save it.
            @param path Output path. If None, file_path is used.
        """
        if path is None:
            self.scene.render.filepath = self.file_path
        else:
            self.scene.render.filepath = path

        bpy.ops.render.render(write_still=True)

    def animation_and_save(self, path=None):
        """!@brief Render video and save it.
            @param path Output path. If None, file_path is used.
        """
        if path is None:
            self.scene.render.filepath = self.file_path
        else:
            self.scene.render.filepath = path

        bpy.ops.render.render(animation=True, write_still=True)
