import itertools
from matplotlib import pyplot as plt
import pybullet as p
from shapes3d import Shapes3D

def run_example():
    """ Simple example to save the rendered image in a folder called toremove"""
    dim = 5
    env = Shapes3D(gui=0, env_dim=dim)
    env.reset()

    env.add_sphere(0.2, [1, 1, 1, 1], [1, 1, 0.2])
    env.add_cube([0.2, 0.2, 0.2], [0, 0, 1, 1], [-1, 1, 0.2])
    env.add_capsule(0.2, 1, [1, 0, 0, 1], [-1, -1, 0.7])
    env.add_cylinder(0.4, 1.5, [1, 0, 0, 1], [1, -1, 0.75])

    intrinsic = p.computeProjectionMatrixFOV(fov=45,
                                             aspect=1,
                                             nearVal=.1,
                                             farVal=dim*2)
    for i in itertools.count(0, 1):
        extrinsic = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[1, 1, 1],
            distance=2.5,
            yaw=i*5,
            pitch=-17,
            roll=0,
            upAxisIndex=2)
        img, depth, segm = env.compute_render(320, 320, intrinsic, extrinsic)

        if i % 100 == 0:
            # Save image
            plt.imsave("%i.png" % i, img)

        if i > 1000:
            break

    env.destroy()

if __name__ == '__main__':
    run_example()
