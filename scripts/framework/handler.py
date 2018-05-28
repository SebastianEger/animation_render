import bpy
import flickrapi
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
        self.file_path = file_path

        self._init_world()
        self._init_scene()

        bpy.data.cameras['Camera'].clip_start = 0.001

    def create_animation(self, settings):
        pass

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
        self.camera.rotation_euler = (0, 0, 0)
        self._remove_init_cube()

    def set_background_pil(self, image):
        if not os.path.exists(".tmp"):
            os.makedirs(".tmp")

        image.save(".tmp/background.png")
        self.bgr_tex.image = bpy.data.images.load(".tmp/background.png")

        image_size = self.bgr_tex.image.size
        self.set_resolution(image_size[0], image_size[1])

    def set_background(self, image):
        self.bgr_tex.image = bpy.data.images.load(image)
        image_size = self.bgr_tex.image.size
        self.set_resolution(image_size[0], image_size[1])

    def set_horizon_color(self, color):
        self.world.horizon_color = color

    def use_anti_aliasing(self, b):
        self.scene.render.use_bake_antialiasing = b
        self.scene.render.use_antialiasing = b

    @staticmethod
    def set_resolution(x, y):
        bpy.context.scene.render.resolution_x = x
        bpy.context.scene.render.resolution_y = y
        # print("Set render resolution to " + str(x) + "x" + str(y))

    def set_fps(self, fps):
        self.scene.render.fps = fps

    def set_focal_length(self, focal_length):
        bpy.data.cameras['Camera'].lens = focal_length

    def set_sensor_width(self, sensor_width):
        bpy.data.cameras['Camera'].sensor_width = sensor_width

    def set_interpolation(self, interpol):
        actions = bpy.data.actions
        for action in actions:
            fcurves = action.fcurves
            for fcurve in fcurves:
                for kf in fcurve.keyframe_points:
                    kf.interpolation = interpol

    def use_environment_light(self, b):
        self.world.light_settings.use_environment_light = b

    def render_and_save(self, path=None):
        if path is None:
            self.scene.render.filepath = self.file_path
        else:
            self.scene.render.filepath = path

        bpy.ops.render.render(write_still=True)

    def animation_and_save(self, path=None):
        if path is None:
            self.scene.render.filepath = self.file_path
        else:
            self.scene.render.filepath = path

        bpy.ops.render.render(animation=True, write_still=True)


class TestHandler:

    img_handler = None
    bdr_handler = None
    params = None

    def __init__(self, blender_handler, params):
        self.params = params

        # init image handler
        self.img_handler = ImageHandler(params['api_key'], params['api_secret'], params['n_images'])
        self.bdr_handler = blender_handler

    def run_tests(self, main_object):
        # self.img_handler.load_photos_same_keywords(self.params['keywords'])
        self.img_handler.load_photos_different_keywords(self.params["background_keywords"],
                                                        self.params["template_keywords"])

        for n_run in range(self.params['n_images']):
            tpl_img = self.img_handler.get_template()
            bgr_img = self.img_handler.get_background()

            self.bdr_handler.set_background_pil(bgr_img)
            main_object.set_image(tpl_img)

            # self.bdr_handler.render_and_save("render/render" + str(n_run))
            self.bdr_handler.animation_and_save("render/render" + str(n_run))


class ImageHandler:
    api = None
    api_key = None
    api_secret = None

    tpl_photos = list()
    bgr_photos = list()

    n_images = 100

    def __init__(self, api_key, api_secret, n_images):
        self.api_key = api_key
        self.api_secret = api_secret
        self.n_images = n_images

        # init api
        self.api = flickrapi.FlickrAPI(self.api_key, self.api_secret, cache=True)

    def load_photos_same_keywords(self, keywords):

        self.tpl_photos.clear()
        self.bgr_photos.clear()

        photos_gen = self.api.walk(text=keywords,
                                   tag_mode='all',
                                   tags=keywords,
                                   extras='url_c',
                                   per_page=100)

        for photo in photos_gen:

            if photo.get('url_c') is None:
                continue
            elif len(self.tpl_photos) < self.n_images:
                self.tpl_photos.append(photo)
            elif len(self.bgr_photos) < self.n_images:
                self.bgr_photos.append(photo)
            else:
                break

        print("Loaded " + str(len(self.tpl_photos)) + " template photos.")
        print("Loaded " + str(len(self.bgr_photos)) + " background photos.")

    def load_photos_different_keywords(self, bgr_keywords, tpl_keywords):
        self.tpl_photos.clear()
        self.bgr_photos.clear()

        photos_bgr_gen = self.api.walk(text=bgr_keywords,
                                       tag_mode='all',
                                       tags=bgr_keywords,
                                       extras='url_c',
                                       per_page=100)

        for photo in photos_bgr_gen:
            if photo.get('url_c') is None:
                continue
            elif len(self.bgr_photos) < self.n_images:
                self.bgr_photos.append(photo)
            else:
                break

        photos_tpl_gen = self.api.walk(text=tpl_keywords,
                                       tag_mode='all',
                                       tags=tpl_keywords,
                                       extras='url_c',
                                       per_page=100)

        for photo in photos_tpl_gen:
            if photo.get('url_c') is None:
                continue
            elif len(self.tpl_photos) < self.n_images:
                self.tpl_photos.append(photo)
            else:
                break

        print("Loaded " + str(len(self.tpl_photos)) + " template photos.")
        print("Loaded " + str(len(self.bgr_photos)) + " background photos.")

    def get_background(self):
        url = self.bgr_photos[0].get("url_c")
        self.bgr_photos.pop(0)

        return self._load_image(url)

    def get_template(self):
        url = self.tpl_photos[0].get("url_c")
        self.tpl_photos.pop(0)

        return self._load_image(url)

    @staticmethod
    def _load_image(url):
        from urllib import request
        data = request.urlopen(url).read()

        import io
        stream = io.BytesIO(data)

        return Image.open(stream)
