import math
import random

"""@package Animation
Contains the base Animation class and all animations.
"""
class Animation:

    obj = None
    interpolation = "QUAD"

    def __init__(self, bdr_handler, blender_object, frames):
        self.obj = blender_object
        bdr_handler.scene.render.image_settings.file_format = 'AVI_JPEG'
        self._add_keyframes(bdr_handler.scene, blender_object, frames)
        bdr_handler.scene.frame_start = 1
        bdr_handler.scene.frame_end = frames
        bdr_handler.scene.frame_set(1)

    def _add_keyframes(self, scene, blender_object, frames):
        raise NotImplementedError

    @staticmethod
    def get_start_position():
        """ Returns the init position """
        return (0, 0, 0)

    @staticmethod
    def get_start_rotation():
        """ Returns the init rotation """
        return (0, 0, 0)


class Example(Animation):
    # BEZIER LINEAR QUAD CUBIC QUART
    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        # Init location and rotation
        blender_object.set_location((0, 0, 0))
        blender_object.set_rotation((0, 0, 0))

        # Initial Time
        blender_object.keyframe_insert("location", 1)
        blender_object.keyframe_insert("location", int(frames/5))

    @staticmethod
    def get_start_position():
        return 0, 0, 0

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0


class RandomRotation(Animation):

    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)

        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

        # Noise
        action = blender_object.obj.animation_data.action

        for fcu in action.fcurves:
            if fcu.data_path == "rotation_euler":  # and fcu.array_index == 0:
                mod = fcu.modifiers.new("NOISE")
                mod.strength = 1
                mod.scale = 20
                # mod.phase = random.uniform(0, 100)
                mod.phase = fcu.array_index*100
                mod.use_restricted_range = True
                mod.frame_start = 5
                mod.frame_end = int(frames)
                mod.blend_in = 100


    @staticmethod
    def get_start_position():
        return 0, 0, 1.5

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0


class RandomMovement(Animation):

    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)

        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

        # Noise
        action = blender_object.obj.animation_data.action

        for fcu in action.fcurves:
            if fcu.data_path == "location":  # and fcu.array_index == 0:
                mod = fcu.modifiers.new("NOISE")
                mod.strength = 0.1
                mod.scale = 20
                # mod.phase = random.uniform(0, 100)
                mod.phase = fcu.array_index*50
                mod.use_restricted_range = True
                mod.frame_start = 5
                mod.frame_end = int(frames)
                mod.blend_in = 100


    @staticmethod
    def get_start_position():
        return 0, 0, 1.5

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0


class RandomMixed(Animation):

    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)

        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

        # Noise
        action = blender_object.obj.animation_data.action

        for fcu in action.fcurves:
            if fcu.data_path == "location":  # and fcu.array_index == 0:
                mod = fcu.modifiers.new("NOISE")
                mod.strength = 0.1
                mod.scale = 40
                # mod.phase = random.uniform(0, 100)
                mod.phase = fcu.array_index*50
                mod.use_restricted_range = True
                mod.frame_start = 5
                mod.frame_end = int(frames)
                mod.blend_in = 100
            if fcu.data_path == "rotation_euler":  # and fcu.array_index == 0:
                mod = fcu.modifiers.new("NOISE")
                mod.strength = 1
                mod.scale = 40
                # mod.phase = random.uniform(0, 100)
                mod.phase = fcu.array_index*100
                mod.use_restricted_range = True
                mod.frame_start = 5
                mod.frame_end = int(frames)
                mod.blend_in = 100

    @staticmethod
    def get_start_position():
        return 0, 0, 1.5

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0


class RotationX(Animation):

    interpolation = 'QUAD'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (-math.pi/2+0.1, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)

        blender_object.set_rotation( (math.pi/2-0.1, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))


    @staticmethod
    def get_start_position():
        return 0, 0, 1.5

    @staticmethod
    def get_start_rotation():
        return -math.pi/2+0.1, 0, 0


class RotationX2(Animation):

    interpolation = 'QUAD'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)

        blender_object.set_rotation( (math.pi/2, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))


    @staticmethod
    def get_start_position():
        return 0, 0, 1.5

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0


class RotationXacc(Animation):

    interpolation = 'QUAD'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (-0.9*math.pi, 0, 0) )
        blender_object.set_location( (0, 0, 1) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)

        blender_object.set_rotation( (0.9*math.pi, 0, 0) )
        blender_object.set_location( (0, 0, 1) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))


    @staticmethod
    def get_start_position():
        return 0, 0, 1

    @staticmethod
    def get_start_rotation():
        return -0.9*math.pi, 0, 0


class RotationY(Animation):

    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)


        blender_object.set_rotation( (0, math.pi/2, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))


    @staticmethod
    def get_start_position():
        return 0, 0, 1.5

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0

class RotationZ(Animation):

    interpolation = 'QUAD'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)


        blender_object.set_rotation( (0, 0, math.pi/2) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))


    @staticmethod
    def get_start_position():
        return 0, 0, 1.5

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0


class TranslationXY(Animation):

        interpolation = 'LINEAR'

        def __init__(self, bdr_handler, blender_object, frames):
            Animation.__init__(self, bdr_handler, blender_object, frames)

        def _add_keyframes(self, scene, blender_object, frames):
            blender_object.set_rotation( (-math.pi/2, 0, 0) )
            blender_object.set_location( (-0.3, 0.3, 1) )
            blender_object.keyframe_insert("rotation_euler", 1)
            blender_object.keyframe_insert("location", 1)


            blender_object.set_rotation( (-math.pi/2, 0, 0) )
            blender_object.set_location( (0.3, 0, 1) )
            blender_object.keyframe_insert("rotation_euler", int(frames))
            blender_object.keyframe_insert("location", int(frames))

        @staticmethod
        def get_start_rotation():
            return -math.pi/2, 0, 0

        @staticmethod
        def get_start_position():
            return -0.3, 0.3, 1


class TranslationXYrotated(Animation):

    interpolation = 'QUAD'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (math.pi/4, 0, 0) )
        blender_object.set_location( (-0.2, -0.2, 1) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)


        blender_object.set_rotation( (math.pi/4, 0, 0) )
        blender_object.set_location( (0.2, 0.2, 1) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_position():
        return -0.2, -0.2, 1

    @staticmethod
    def get_start_rotation():
        return math.pi/4, 0, 0


class TranslationY(Animation):

    interpolation = 'QUAD'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (math.pi/4, 0, 0) )
        blender_object.set_location( (0, -0.5, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)


        blender_object.set_rotation( (math.pi/4, 0, 0) )
        blender_object.set_location( (0, 0.5, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_rotation():
        return math.pi/4, 0, 0

    @staticmethod
    def get_start_position():
        return 0, -0.5, 1.5


class TranslationX(Animation):

    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (-math.pi/2, 0, 0) )
        blender_object.set_location( (-0.3, 0.3, 1) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)


        blender_object.set_rotation( (-math.pi/2, 0, 0) )
        blender_object.set_location( (0.3, 0.3, 1) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_rotation():
        return -math.pi/2, 0, 0

    @staticmethod
    def get_start_position():
        return -0.3, 0.3, 1


class TranslationX2(Animation):

    interpolation = 'QUAD'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (-0.5, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)


        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0.5, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0

    @staticmethod
    def get_start_position():
        return -0.5, 0, 1.5

class TranslationZ(Animation):

    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)


        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.2) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_position():
        return 0, 0, 1

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0



class Movement2(Animation):

    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)


        blender_object.set_rotation( (0, math.pi/4, 0) )
        blender_object.set_location( (0, 0, 1) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_position():
        return 0, 0, 1

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0


class Movement3(Animation):

    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (math.pi/8, 0, 0) )
        blender_object.set_location( (0, -0.1, 1) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)

        blender_object.keyframe_insert("rotation_euler", 10)
        blender_object.keyframe_insert("location", 10)

        blender_object.set_rotation( (-math.pi/8, 0, 0) )
        blender_object.set_location( (0, 0.1, 1) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_position():
        return 0, -0.1, 1

    @staticmethod
    def get_start_rotation():
        return math.pi/8, 0, 0

class Movement4(Animation):

    interpolation = 'LINEAR'

    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)


        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0.2, 0.2, 1) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

    @staticmethod
    def get_start_position():
        return 0, 0, 1

    @staticmethod
    def get_start_rotation():
        return 0, 0, 0

class Convergence(Animation):
    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((0, 0, 1))
        blender_object.set_rotation((0, 0, 0))
        blender_object.keyframe_insert("location", 1)
        blender_object.keyframe_insert("rotation_euler", 1)

    @staticmethod
    def get_start_position():
        return 0.0, 0.0, 1.0

    @staticmethod
    def get_start_rotation():
        return 0.1, 0.1, 0.1


class ConvergenceFrame(Animation):
    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((0, 0, 1))
        blender_object.set_rotation((0.1, 0.1, 0.1))
        blender_object.keyframe_insert("location", 1)
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.set_location((0, 0, 1))
        blender_object.set_rotation((0, 0, 0))
        blender_object.keyframe_insert("location", 2)
        blender_object.keyframe_insert("rotation_euler", 2)

    @staticmethod
    def get_start_position():
        return 0.0, 0.0, 1.0

    @staticmethod
    def get_start_rotation():
        return 0.1, 0.1, 0.1


class RenderTest(Animation):
    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_location((0, 0, 1))
        blender_object.set_rotation((0.0, 0.0, 0.0))
        blender_object.keyframe_insert("location", 1)
        blender_object.keyframe_insert("rotation_euler", 1)

    @staticmethod
    def get_start_position():
        return 0.0, 0.0, 0.4

    @staticmethod
    def get_start_rotation():
        return 0.0, 0.0, 0.0


class ForRender(Animation):
    def __init__(self, bdr_handler, blender_object, frames):
        Animation.__init__(self, bdr_handler, blender_object, frames)

    def _add_keyframes(self, scene, blender_object, frames):
        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", 1)
        blender_object.keyframe_insert("location", 1)

        blender_object.set_rotation( (0, 0, 0) )
        blender_object.set_location( (0, 0, 1.5) )
        blender_object.keyframe_insert("rotation_euler", int(frames))
        blender_object.keyframe_insert("location", int(frames))

        # Noise
        action = blender_object.obj.animation_data.action

        for fcu in action.fcurves:
            if fcu.data_path == "rotation_euler" and fcu.array_index == 0: # x
                mod = fcu.modifiers.new("NOISE")
                mod.strength = 4
                mod.scale = 40
                # mod.phase = random.uniform(0, 100)
                mod.phase = fcu.array_index*100
                mod.use_restricted_range = True
                mod.frame_start = 5
                mod.frame_end = int(frames)
                mod.blend_in = 100

    @staticmethod
    def get_start_position():
        return 0.0, 0.0, 1.5

    @staticmethod
    def get_start_rotation():
        return 0.0, 0.0, 0.0
