import math


class Animation:

    def __init__(self, bdr_handler, blender_object, frames):
        bdr_handler.scene.render.image_settings.file_format = 'AVI_JPEG'
        self._add_keyframes(bdr_handler.scene, blender_object, frames)
        bdr_handler.scene.frame_start = 1
        bdr_handler.scene.frame_end = frames
        bdr_handler.scene.frame_set(1)

    def _add_keyframes(self, scene, blender_object, frames):
        raise NotImplementedError

    @staticmethod
    def get_start_position():
        return (0, 0, 0)

    @staticmethod
    def get_start_rotation():
        return (0, 0, 0)


class RotationPlaneZ(Animation):

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((0, 0, -0.10))
        blender_object.keyframe_insert("rotation_euler", 1)
        # init time
        blender_object.keyframe_insert("rotation_euler", int(frames/5))

        blender_object.set_rotation((0, 0, 2*math.pi))

        blender_object.keyframe_insert("rotation_euler", int(frames))

    @staticmethod
    def get_start_position():
        return (0, 0, -0.10)


class RotationX(Animation):

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((0, 0, -0.10))
        blender_object.keyframe_insert("rotation_euler", 1)
        # init time
        blender_object.keyframe_insert("rotation_euler", int(frames/5))

        blender_object.set_rotation((2*math.pi, 0, 0))

        blender_object.keyframe_insert("rotation_euler", int(frames))

    @staticmethod
    def get_start_position():
        return (0, 0, -0.10)


class TranslationPlane1(Animation):

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((-0.03, -0.03, -0.15))

        blender_object.keyframe_insert("location", 1)
        blender_object.keyframe_insert("location", int(frames/5))

        blender_object.keyframe_insert("location", int(frames/4))

        blender_object.set_location((0.03, -0.03, -0.15))

        blender_object.keyframe_insert("location", int(frames/2))

        blender_object.set_location((0.03, 0.03, -0.15))

        blender_object.keyframe_insert("location", int(frames*3/4))

        blender_object.set_location((-0.03, 0.03, -0.15))

        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_position():
        return (-0.03, -0.03, -0.15)



class RotationCylinder1(Animation):

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((0, 0, -0.10))
        blender_object.set_rotation((0, -math.pi/2, 0))

        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("rotation_euler", int(frames/5))

        blender_object.set_rotation((0, -math.pi/2, 2*math.pi))
        blender_object.keyframe_insert("rotation_euler", int(frames))


    @staticmethod
    def get_start_position():
        return (0, 0, -0.10)

    @staticmethod
    def get_start_rotation():
        return (0, -math.pi/2, 0)


class RotationCylinder2(Animation):

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((0, 0, -0.10))
        blender_object.set_rotation((0, -math.pi/2, 0.5))

        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("rotation_euler", int(frames/5))

        blender_object.set_rotation((2*math.pi, -math.pi/2, 0.5))
        blender_object.keyframe_insert("rotation_euler", int(frames))

    @staticmethod
    def get_start_position():
        return (0, 0, -0.10)

    @staticmethod
    def get_start_rotation():
        return (0, -math.pi/2, 0.5)


class TranslationCylinder1(Animation):

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((-0.03, -0.03, -0.15))
        blender_object.set_rotation((0, 0, 0))

        blender_object.keyframe_insert("location", 1)
        blender_object.keyframe_insert("location", int(frames/5))

        blender_object.keyframe_insert("location", int(frames/4))

        blender_object.set_location((0.03, -0.03, -0.15))

        blender_object.keyframe_insert("location", int(frames/2))

        blender_object.set_location((0.03, 0.03, -0.15))

        blender_object.keyframe_insert("location", int(frames*3/4))

        blender_object.set_location((-0.03, 0.03, -0.15))

        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_position():
        return -0.03, -0.03, -0.15

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0


class TestAdvanceFaceTracker(Animation):

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((0, 0, -0.15))
        blender_object.set_rotation((0.0, 0, 0))

        blender_object.keyframe_insert("location", 1)
        blender_object.keyframe_insert("rotation_euler", 1)

        blender_object.set_rotation((0.5 , 0, 0))
        blender_object.keyframe_insert("location", int(frames))
        blender_object.keyframe_insert("rotation_euler", int(frames))

    @staticmethod
    def get_start_position():
        return 0.0, 0.0, -0.15

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0

