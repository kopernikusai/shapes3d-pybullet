from shapes3d import Shapes3D
import itertools
import datetime
import pybullet as p
import time

def run_example():
    env = Shapes3D(gui=0, env_dim=5)
    env.reset()

    env.add_sphere(0.2,[1,1,1,1],[1,1,0.2])
    env.add_cube([0.2,0.2,0.2],[0,0,1,1],[-1,1,0.2])
    env.add_capsule(0.2,1,[1,0,0,1], [-1,-1,0.7])
    env.add_cylinder(0.4, 1.5, [1,0,0,1], [1,-1,0.75])

    intrinsic = p.computeProjectionMatrixFOV(fov=45,
                                             aspect=1,
                                             nearVal=.1,
                                             farVal=env._env_dim*2)
    for i in itertools.count(0,1):
        start = time.time()
        extrinsic = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[1,1,1],
                distance=2.5,
                yaw=i*5,
                pitch=-17,
                roll=0,
                upAxisIndex=2)
        env.compute_render(320,320,intrinsic,extrinsic)
        end = time.time()
        print("rendering took: %f" % (end - start))
        if i > 1000: break

    env.reset() # Not needed, testing purposes only
    env.destroy()

if __name__ == '__main__':
    a = datetime.datetime.now()
    run_example()
    b = datetime.datetime.now()
    print((b-a).total_seconds())
