import bpy, bmesh, math


class BlenderObject:

    name = None
    obj = None  # object
    img = None  # image
    tex = None  # texture
    mat = None  # material

    def __init__(self, name, image=None):
        self.name = name

        # Create Object
        self.obj = self._create_obj()

        # Add Object to Scene
        bpy.context.scene.objects.link(self.obj)

        # Stripe Material
        self._stripe_material()

        self.obj.data.materials.append(self.mat)

        self._uv()

        if image:
            self.tex.image = bpy.data.images.load(image)

    def set_location(self, loc):
        self.obj.location = loc

    def set_rotation(self, rot):
        self.obj.rotation_euler = rot

    def set_image(self, image):
        if not os.path.exists(".tmp"):
            os.makedirs(".tmp")

        image.save(".tmp/template.png")
        self.tex.image = bpy.data.images.load(".tmp/template.png")

    def keyframe_insert(self, keyframe_type, frame):
        self.obj.keyframe_insert(data_path=keyframe_type, frame=frame)

    def _create_obj(self):
        """ Function to create object """
        raise NotImplementedError

    def _uv(self):
        """ Function to do uv mapping """
        raise NotImplementedError

    def _stripe_material(self):
        # create a texture for the image
        self.tex = bpy.data.textures.new("stripe", 'IMAGE')

        # give it sharp uninterpolated edges.
        self.tex.use_interpolation = False
        self.tex.filter_type = 'BOX'

        # put the texture on a material with UV coordinates
        self.mat = bpy.data.materials.new('stripe')
        self.mat.texture_slots.add()
        self.mat.texture_slots[0].texture = self.tex
        self.mat.texture_slots[0].texture_coords = 'UV'
        self.mat.texture_slots[0].uv_layer = 'uv'


class Cylinder(BlenderObject):

    # class specific
    nFaces = 100
    radius = 1
    length = 5

    top_face = []
    bot_face = []

    mesh = None

    def __init__(self, p1, p2, p3, image=None):

        self.length = float(p1)
        self.radius = float(p2)
        self.nFaces = int(float(p3))

        BlenderObject.__init__(self, "Cylinder", image)
        self.obj.rotation_mode = 'YZX'

    def _create_obj(self):
        # this makes a simple open-ended cylinder.
        mesh = bpy.data.meshes.new(self.name)

        verts = []
        faces = []

        top_face = []
        bot_face = []

        for i in range(self.nFaces):
            # theta = 2*math.pi* i / self.nFaces + math.pi/2
            theta = 2*math.pi* i / self.nFaces
            c = -self.radius * math.sin(theta)
            s = -self.radius * math.cos(theta)
            # verts.append([c, s, self.length / 2])
            # verts.append([c, s, -self.length / 2])
            verts.append([-self.length / 2, c, s])
            verts.append([+self.length / 2, c, s])
            v1 = i * 2
            v2 = v1 + 1
            v3 = v1 + 2
            v4 = v1 + 3
            if v3 >= 2 * self.nFaces:
                v3 = 0
                v4 = 1
            faces.append([v1, v3, v4, v2])
            top_face.append(v1)
            bot_face.append(v2)

        faces.append(top_face)
        faces.append(bot_face)
        mesh.from_pydata(verts, [], faces)
        mesh.validate(True)
        mesh.show_normal_face = True

        self.mesh = mesh

        return bpy.data.objects.new(self.name, mesh)

    def _uv(self):
        self.obj.data.uv_textures.new("uv")

        bm = bmesh.new()
        bm.from_mesh(self.obj.data)
        bm.faces.ensure_lookup_table()

        uv_layer = bm.loops.layers.uv[0]

        for fi in range(self.nFaces):
            x0 = fi  / self.nFaces
            x1 = (fi + 1) / self.nFaces
            bm.faces[fi].loops[0][uv_layer].uv = (0, x0)
            bm.faces[fi].loops[1][uv_layer].uv = (0, x1)
            bm.faces[fi].loops[2][uv_layer].uv = (1, x1)
            bm.faces[fi].loops[3][uv_layer].uv = (1, x0)

        for l in bm.faces[self.nFaces].loops:
            l[uv_layer].uv = (0, 0)

        for l in bm.faces[self.nFaces+1].loops:
            l[uv_layer].uv = (0, 0)

        bm.to_mesh(self.obj.data)


class Plane(BlenderObject):

    width = None
    height = None

    def __init__(self, p1, p2, p3, image=None):
        self.width  = float(p1)
        self.height = float(p2)

        BlenderObject.__init__(self, "Plane", image)
        self.obj.rotation_mode = 'YZX'

    def _uv(self):
        self.obj.data.uv_textures.new("uv")
        bm = bmesh.new()
        bm.from_mesh(self.obj.data)

        bm.faces.ensure_lookup_table()

        uv_layer = bm.loops.layers.uv[0]

        bm.faces[0].loops[0][uv_layer].uv = (1, 1)
        bm.faces[0].loops[1][uv_layer].uv = (0, 1)
        bm.faces[0].loops[2][uv_layer].uv = (0, 0)
        bm.faces[0].loops[3][uv_layer].uv = (1, 0)

        bm.to_mesh(self.obj.data)

    def _create_obj(self):
        # this makes a simple open-ended cylinder.
        mesh = bpy.data.meshes.new(self.name)

        verts = []
        faces = []

        verts.append([self.height / 2, -self.width / 2, 0])
        verts.append([-self.height / 2, -self.width / 2, 0])
        verts.append([-self.height / 2,  self.width / 2, 0])
        verts.append([ self.height / 2,  self.width / 2, 0])

        faces.append([0, 1, 2, 3])
        mesh.from_pydata(verts, [], faces)
        mesh.validate(True)
        mesh.show_normal_face = True

        return bpy.data.objects.new(self.name, mesh)


class Curved(BlenderObject):

    # class specific
    nFaces = 100
    radius = 1
    length = 5

    top_face = []
    bot_face = []

    mesh = None

    def __init__(self, p1, p2, p3, image=None):

        self.length = float(p1)
        self.radius = float(p2)
        self.nFaces = int(float(p3))

        BlenderObject.__init__(self, "Cylinder", image)
        self.obj.rotation_mode = 'YZX'

    def _create_obj(self):
        # this makes a simple open-ended cylinder.
        mesh = bpy.data.meshes.new(self.name)

        verts = []
        faces = []

        top_face = []
        bot_face = []

        for i in range(self.nFaces):
            # theta = 2*math.pi* i / self.nFaces + math.pi/2
            theta = math.pi* i / (self.nFaces-1) - math.pi/2
            c = -self.radius * math.sin(theta)
            s = -self.radius * math.cos(theta)
            # verts.append([c, s, self.length / 2])
            # verts.append([c, s, -self.length / 2])
            verts.append([-self.length / 2, c, s])
            verts.append([+self.length / 2, c, s])
            v1 = i * 2
            v2 = v1 + 1
            v3 = v1 + 2
            v4 = v1 + 3
            if v3 >= 2 * self.nFaces:
                v3 = 0
                v4 = 1
            faces.append([v1, v3, v4, v2])
            top_face.append(v1)
            bot_face.append(v2)

        faces.append(top_face)
        faces.append(bot_face)
        mesh.from_pydata(verts, [], faces)
        mesh.validate(True)
        mesh.show_normal_face = True

        self.mesh = mesh

        return bpy.data.objects.new(self.name, mesh)

    def _uv(self):
        self.obj.data.uv_textures.new("uv")

        bm = bmesh.new()
        bm.from_mesh(self.obj.data)
        bm.faces.ensure_lookup_table()

        uv_layer = bm.loops.layers.uv[0]

        for fi in range(self.nFaces):
            x0 = fi  / self.nFaces
            x1 = (fi + 1) / self.nFaces
            bm.faces[fi].loops[0][uv_layer].uv = (0, x0)
            bm.faces[fi].loops[1][uv_layer].uv = (0, x1)
            bm.faces[fi].loops[2][uv_layer].uv = (1, x1)
            bm.faces[fi].loops[3][uv_layer].uv = (1, x0)

        for l in bm.faces[self.nFaces].loops:
            l[uv_layer].uv = (0, 0)

        for l in bm.faces[self.nFaces+1].loops:
            l[uv_layer].uv = (0, 0)

        bm.to_mesh(self.obj.data)
