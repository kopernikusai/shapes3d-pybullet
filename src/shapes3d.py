import os
import pkgutil
import pybullet as p
import pybullet_data

MASS = 1
GRAVITY = -9.8
PLANE_MASS = 0 # Unmovable object
MAX_OBJ_DIM = 10 # Maximum object dimension

class Shapes3D():
    """
        Defines the basic structure of the Shapes3D environement

        gui (bool): Controlls if initialize the pybullet simulation using gui or not

        set_colliders (bool): Controlls if each body created in the environment
        should also include a colliderShape

        env_dims (int): the dimensions of the environment, no object can be outside this dim

        use_egl_plugin (bool): set it to true if you want to use the egl plugin to render much
        faster. If not specified it will be used if possible. This pluging helps rendering the
        image without a X11 context in Linux based systems. Warning EGL tends to render un-wanted
        artifacts such as the axis
    """
    def __init__(self, gui=False, use_egl_plugin=False, env_dim=10):

        if env_dim <= 0:
            raise AttributeError("Ivalid env_dim passed to initialization of Shape3D env")

        if not ((use_egl_plugin and not gui) or use_egl_plugin in [None, False]):
            raise AttributeError("Ivalid type for use_egl_plugin, it should be \
                    boolean or gui==False")

        self._plugin = None
        # Check if we can load egl plugin or throw an error if user specified it
        if (use_egl_plugin is None and not gui) or use_egl_plugin:
            self._egl = pkgutil.get_loader('eglRenderer')
            if self._egl is not None:
                self.use_egl_plugin = True
                del os.environ["DISPLAY"]
                print("Using EGL Plugin to accelerate rendering")
            elif use_egl_plugin:
                raise ImportError("Cannot load eglRenderer")
        else:
            self.use_egl_plugin = False

        self.gui = p.GUI if gui else p.DIRECT
        self._env_dim = env_dim

        self.objects_id = []
        self.physics_client = None

        self.plane_id = None

    def reset(self):
        """ reset the environment to the orignal state, i.e. an empty environment """
        if self.physics_client is None:
            self.physics_client = p.connect(self.gui)
            p.setGravity(0, 0, GRAVITY)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())

            # Loading egl plugin for faster rendering
            if self.use_egl_plugin:
                self._plugin = p.loadPlugin(self._egl.get_filename(), '_eglRendererPlugin')

            if self.gui != p.GUI:
                p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

            # Create plane, the plan urdf has original dims of 200
            self.plane_id = p.loadURDF('plane100.urdf', globalScaling=self._env_dim/200)
            p.changeVisualShape(self.plane_id, -1, textureUniqueId=-1, rgbaColor=[0.6, 0.6, 0.6, 1])

        # Remove objects
        while self.objects_id:
            p.removeBody(self.objects_id[0])
            del self.objects_id[0]

        assert not self.objects_id
        assert p.getNumBodies() == 1

    def add_sphere(self, radius, color, position, orientation=None):
        """
            Adds a sphere with the given radius, color, position and orientation. This function
            checks if the values are valid
        """
        if orientation is None:
            orientation = [0, 0, 0]

        # Check for color and orientation
        self.check_color_orientation(color, position, orientation)
        orientation = p.getQuaternionFromEuler(orientation)

        # Check for valid radius
        if not (radius * 2 < MAX_OBJ_DIM and radius > 0):
            raise AttributeError("Ivalid radius passed to add_sphere")

        # Check for valid pos
        if not position[2] >= radius:
            raise AttributeError("Ivalid position passed to add_sphere")

        # Check for valid pos
        valid_pos = lambda p: abs(p)+radius <= self._env_dim/2
        if not (len(position) == 3 and all(map(valid_pos, position))):
            raise AttributeError("Ivalid position passed to add_sphere")

        visual_shp = p.createVisualShape(p.GEOM_SPHERE,
                                         radius=radius,
                                         rgbaColor=color)
        collision_shp = p.createCollisionShape(p.GEOM_SPHERE,
                                               radius=radius)
        obj_id = p.createMultiBody(MASS,
                                   collision_shp,
                                   visual_shp,
                                   basePosition=position,
                                   baseOrientation=orientation)
        self.objects_id.append(obj_id)
        return obj_id

    def load_urdf(self, urdf, scaling, position, orientation, color):
        """ Function to directly load URDF. It does not run any check of attributes """
        obj_id = p.loadURDF(urdf,
                            globalScaling=scaling,
                            basePosition=position,
                            baseOrientation=orientation)
        p.changeVisualShape(obj_id, -1, textureUniqueId=-1, rgbaColor=color)
        self.objects_id.append(obj_id)
        return obj_id

    def add_cube(self, dimensions, color, position, orientation=None):
        """
            Adds a sphere with the giben dim, color, position and orientation. This
            function checks if the values are valid
        """
        if orientation is None:
            orientation = [0, 0, 0]

        # Check for color and orientation
        self.check_color_orientation(color, position, orientation)
        orientation = p.getQuaternionFromEuler(orientation)

        valid_dim = lambda d: d > 0 and d <= MAX_OBJ_DIM
        if not all(map(valid_dim, dimensions)):
            raise AttributeError("Invalid dimensions passed to add_cube")

        # Check for valid pos
        valid_pos = lambda p: \
                abs(p)+dimensions[0] <= self._env_dim/2 and \
                abs(p)+dimensions[1] <= self._env_dim/2
        if not (len(position) == 3 and all(map(valid_pos, position))):
            raise AttributeError("Ivalid position passed to add_cube")

        visual_shp = p.createVisualShape(p.GEOM_BOX, halfExtents=dimensions, rgbaColor=color)
        collision_shp = p.createCollisionShape(p.GEOM_BOX, halfExtents=dimensions)
        obj_id = p.createMultiBody(MASS,
                                   collision_shp,
                                   visual_shp,
                                   basePosition=position,
                                   baseOrientation=orientation)
        self.objects_id.append(obj_id)
        return obj_id

    def add_capsule(self, radius, length, color, position, orientation=None):
        """ Adds a capsule with the given radius, length, color, position and orientation """
        return self._add_capsule_cylinder(p.GEOM_CAPSULE,
                                          radius,
                                          length,
                                          color,
                                          position,
                                          orientation)

    def add_cylinder(self, radius, length, color, position, orientation=None):
        """ Adds a cylinder with the given radius, length, color, position and orientation """
        return self._add_capsule_cylinder(p.GEOM_CYLINDER,
                                          radius,
                                          length,
                                          color,
                                          position,
                                          orientation)

    def _add_capsule_cylinder(self, obj_type, radius, length, color, position, orientation=None):
        if orientation is None:
            orientation = [0, 0, 0]

        # Check for color and orientation
        self.check_color_orientation(color, position, orientation)
        orientation = p.getQuaternionFromEuler(orientation)

        if not (radius > 0 and radius*2 < MAX_OBJ_DIM):
            raise AttributeError("Ivalid radius passed to _add_capsule_cylinder")

        if not (length > 0 and length+radius <= MAX_OBJ_DIM):
            raise AttributeError("Ivalid length passed to _add_capsule_cylinder")

        valid_pos = lambda p: abs(p)+radius <= self._env_dim/2
        if not all(map(valid_pos, position)):
            raise AttributeError("Ivalid position passed to _add_capsule_cylinder")

        visual_shp = p.createVisualShape(obj_type, radius=radius, length=length, rgbaColor=color)
        collision_shp = p.createCollisionShape(obj_type, radius=radius, height=length)
        obj_id = p.createMultiBody(MASS,
                                   collision_shp,
                                   visual_shp,
                                   basePosition=position,
                                   baseOrientation=orientation)
        self.objects_id.append(obj_id)
        return obj_id

    def remove_object(self, object_id):
        """ Removes the object with id object_id from the environment """
        p.removeBody(object_id)
        self.objects_id.remove(object_id)

    def render(self, width, height, position, distance, yaw, pitch, roll, fov=45., aspect=1):
        """
            This functions renders the scene from the camera parameters.

            Warnings:
                * initialize function using use_egl_plugin=True
                * if you need call this function several times it is faster to
                compute the intrinsic matrix (only once) and re-use using the
                function compute_render (re-use the extrinsic matrix if
                you can to make the rendering faster).

            width, height dim of image (in pixels)
        """
        # Compute view/extrinsic matrix
        extrinsic = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=position,
                                                        distance=distance,
                                                        yaw=yaw,
                                                        pitch=pitch,
                                                        roll=roll,
                                                        upAxisIndex=2)
        # Compute projection/intrinsic matrix
        intrinsic = p.computeProjectionMatrixFOV(fov=fov,
                                                 aspect=aspect,
                                                 nearVal=0.1,
                                                 farVal=self._env_dim*2)
        return self.compute_render(width, height, intrinsic, extrinsic)

    def compute_render(self, width, height, intrinsic, extrinsic):
        """
            This functions creates the image of size width height in pixels from the
            intrinsic and extrinsic matrix
        """
        _, _, img, depth, segmentation = p.getCameraImage(width,
                                                          height,
                                                          viewMatrix=extrinsic,
                                                          projectionMatrix=intrinsic,
                                                          lightDirection=[0, 0, 10],
                                                          lightDistance=100,
                                                          lightColor=[1, 1, 1],
                                                          shadow=1)
        return img, depth, segmentation

    def check_color_orientation(self, color, position, orientation):
        """
            Checks if the color and orientationentation are valid values. It
            also checks if the position is valid, but position can not be valid
            in certain cases depending on the roation and shape of object, this
            is only a basic check.
        """
        # Check for valid color
        valid_color = lambda c: c >= 0 and c <= 1
        if not (len(color) == 4 and all(map(valid_color, color))):
            raise AttributeError("Ivalid color passed to _add_object")

        # Check for valid orientation
        if not len(orientation) == 3:
            raise AttributeError("Ivalid orientation passed to _add_object")

    def destroy(self):
        """ Destroys the environment properly """

        if self.use_egl_plugin:
            p.unloadPlugin(self._plugin)

if __name__ == '__main__':
    pass
