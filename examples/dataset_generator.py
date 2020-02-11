""" Defines an example to create a dataset of an environment """

import argparse
import os
import random
from collections import namedtuple
from pdb import set_trace
from PIL import Image
from shapes3d import Shapes3D

ENV_DIM = 20
MAX_NUM_OBJS = 15
MIN_NUM_OBJS = 7
MIN_OBJ_DIM = 0.5
MAX_OBJ_DIM = 1.5
FOV = 45
PLANE_COLOR = [1, 1, 1]
TYPES = ['sphere', 'cube', 'cylinder', 'capsule']

Obj = namedtuple("Object", "x y collision_radius")

def generate_dataset(destination_folder, num_total_imgs, num_total_envs, num_eval_envs, width, height):
    """
        Generates the images and saves them in destination. It will
        create imgs/envs per envs.
    """

    train, val = [], []

    # Check if folder exists
    check_folder_or_create(destination_folder)

    # Create general environment
    env = Shapes3D(gui=True, use_egl_plugin=False, env_dim=ENV_DIM, walls=True)
    env.reset()

    # Get instrinsic and extrinsic matrices
    intrinsic = env.computeProjectionMatrixFOV(fov=FOV, aspect=1)
    save_in_txt(os.path.join(destination_folder, "intrinsic_matrix.txt"), intrinsic)

    for env_num in range(0, num_total_envs):
        objs = [] # List of positions and dimenssion of objects

        # Create folder if does not exits
        check_folder_or_create(os.path.join(destination_folder, str(env_num)))

        # Create objects
        # Choose number of objects
        num_obj = random.randint(MIN_NUM_OBJS, MAX_NUM_OBJS)
        for _ in range(num_obj):
            # Choose random color
            r = random.uniform(0, 1)
            g = random.uniform(0, 1)
            b = random.uniform(0, 1)
            color = [r, g, b, 1]

            # Choose random type
            obj_type = random.choice(TYPES)

            collision_radius = None
            dimension = None
            # Choose random dims
            if obj_type == "cube":
                x = random.uniform(MIN_OBJ_DIM, MAX_OBJ_DIM)/2
                y = random.uniform(MIN_OBJ_DIM, MAX_OBJ_DIM)/2
                z = random.uniform(MIN_OBJ_DIM, MAX_OBJ_DIM)/2
                dimension = [x, y, z]
                collision_radius = max([x, y])
                z_pos = z
            else:
                dimension = random.uniform(MIN_OBJ_DIM, MAX_OBJ_DIM)
                if obj_type in ["cylinder", "capsule"]:
                    collision_radius = random.uniform(MIN_OBJ_DIM, MAX_OBJ_DIM)
                    z_pos = dimension/2
                else:
                    collision_radius = dimension
                    z_pos = collision_radius

            # Choose random position
            while True:
                min_pos = ENV_DIM/2 - collision_radius
                x_pos = random.uniform(-min_pos, min_pos)
                y_pos = random.uniform(-min_pos, min_pos)

                # Check if objects are far enough
                if check_collisions(x_pos, y_pos, collision_radius, objs):
                    objs.append(Obj(x=x_pos, y=y_pos, collision_radius=collision_radius))
                    break

            position = [x_pos, y_pos, z_pos]
            if obj_type == "cube":
                env.add_cube(dimension, color, position)
            elif obj_type == "sphere":
                env.add_sphere(dimension, color, position)
            elif obj_type == "capsule":
                env.add_capsule(collision_radius, dimension, color, position)
            elif obj_type == "cylinder":
                env.add_cylinder(collision_radius, dimension, color, position)

        # Choose a random pose and orientation for camera
        for img_num in range(int(num_total_imgs/num_total_envs)):
            epsilon = 0.1
            range_dis = ENV_DIM / 2 - epsilon
            while True:
                succ1, img1, depth1, _, pos, ori = get_cam_render(
                    [0, 0, 0],
                    [[-range_dis, range_dis], [-range_dis, range_dis], [MIN_OBJ_DIM, MAX_OBJ_DIM]],
                    [0, 0, 0],
                    [[0, 0], [0, 10], [0, 360]],
                    width, height,
                    intrinsic,
                    env, objs)

                if succ1 is False:
                    continue

                x_cam, y_cam, z_cam = pos
                roll, pitch, yaw = ori

                # Move the camera
                for _ in range(20):
                    succ2, img2, depth2, _, dpos2, dori2 = get_cam_render(
                        [x_cam, y_cam, z_cam], [[-MIN_OBJ_DIM, MIN_OBJ_DIM]] * 3,
                        [roll, pitch, yaw], [[0, 0], [-5, 5], [-5, 5]],
                        width, height,
                        intrinsic,
                        env, objs)

                    if succ2 is True:
                        dx2, dy2, dz2 = dpos2
                        droll2, dpitch2, dyaw2 = dori2

                        # Second movement
                        for _ in range(20):
                            succ3, img3, depth3, _, dpos3, dori3 = get_cam_render(
                                [x_cam - dx2, y_cam - dy2, z_cam - dz2],
                                [[-MIN_OBJ_DIM, MIN_OBJ_DIM]] * 3,
                                [roll, pitch, yaw], [[0, 0], [-5, 5], [-5, 5]],
                                width, height,
                                intrinsic,
                                env, objs)
                            if succ3 is True:
                                dx3, dy3, dz3 = dpos3
                                droll3, dpitch3, dyaw3 = dori3
                                break

                        if succ2 and succ3:
                            break

                if succ2 and succ3:
                    break

            # Save the picture
            dest = os.path.join(destination_folder, str(env_num))
            img_dest   = os.path.join(dest, '%s.jpg')
            depth_dest = os.path.join(dest, '%s.npy')
            dof_dest   = os.path.join(dest, "%s_6dof.txt")

            # Imgs
            img1 = Image.fromarray(img1[:, :, :3])
            img2 = Image.fromarray(img2[:, :, :3])
            img3 = Image.fromarray(img3[:, :, :3])
            img2.save(img_dest % (str(img_num) + "_a"))
            img1.save(img_dest % (str(img_num) + "_b"))
            img3.save(img_dest % (str(img_num) + "_c"))

            # Depths
            depth2.dump(depth_dest % (str(img_num) + "_a"))
            depth1.dump(depth_dest % (str(img_num) + "_b"))
            depth3.dump(depth_dest % (str(img_num) + "_c"))

            if env_num < num_eval_envs:
                val.append((str(env_num), str(img_num)))
            else:
                train.append((str(env_num), str(img_num)))

            # Extrinsic
            save_in_txt(dof_dest % (str(img_num) + "_btoa"),
                    [dx2, dy2, dz2, droll2, dpitch2, dyaw2])
            save_in_txt(dof_dest % (str(img_num) + "_btoc"),
                    [dx3 - dx2, dy3 - dy2, dz3 - dz2, droll3, dpitch3, dyaw3])

        # Clean environment
        env.reset()

    # Save split values
    train = random.sample(train, len(train))
    val = random.sample(val, len(val))

    train = [' '.join(x) for x in train]
    val = [' '.join(x) for x in val]

    save_in_txt(os.path.join(destination_folder, "train.txt"), train, '\n')
    save_in_txt(os.path.join(destination_folder, "val.txt"), val, '\n')

def get_cam_render(pos0, pos_range, ori0, ori_range, width, height, intrinsic, env, objs):
    """ Creates a render based on arguments and checks if img contains objs """
    distance = 1e-5

    img, depth, seg, dpos, dori = [None]*5
    x_cam, y_cam, z_cam = pos0
    x_rang, y_rang, z_rang = pos_range
    roll, pitch, yaw = ori0
    roll_rang, pitch_rang, yaw_rang = ori_range

    dx = random.uniform(*x_rang)
    dy = random.uniform(*y_rang)
    dz = random.uniform(*z_rang)

    x_cam2 = x_cam + dx
    y_cam2 = y_cam + dy
    z_cam2 = z_cam + dz
    position = [x_cam2, y_cam2, z_cam2]

    # Check position
    if not check_collisions(x_cam2, y_cam2, MIN_OBJ_DIM*2, objs):
        return False, img, depth, seg, dpos, dori

    # Check position
    if abs(x_cam2) > ENV_DIM/2 or abs(y_cam2) > ENV_DIM/2:
        return False, img, depth, seg, dpos, dori

    droll = random.uniform(*roll_rang)
    dpitch = random.uniform(*pitch_rang)
    dyaw = random.uniform(*yaw_rang)

    roll2 = roll + droll
    pitch2 = pitch + dpitch
    yaw2 = yaw + dyaw

    # Render
    extrinsic = env.computeViewMatrixFromYawPitchRoll(
        position, distance, yaw2, pitch2, roll2)
    img, depth, seg = env.compute_render(width, height, intrinsic, extrinsic)

    # Check if position renders at least two objects
    seg = set(seg.reshape(1, -1).tolist()[0]) # Removing repeations
    seg = seg.difference([-1]+env.walls_id) # Removing plane_id
    if env.plane_id in seg and len(seg) > 3:
        dpos = [dx, dy, dz]
        dori = [droll, dpitch, dyaw]
        return True, img, depth, seg, dpos, dori
    else:
        return False, img, depth, seg, dpos, dori

def save_in_txt(destination, array, char=' '):
    """ saves array elements in destination with spaces between values """
    if os.path.exists(destination):
        os.remove(destination)
    with open(destination, 'w') as txt:
        txt.write(char.join([str(i) for i in array]))

def check_collisions(x, y, collision_radius, objs):
    """
        Checks if the position x,y collide with any obj in objs with a
        collision_raidus of collision_radius
    """
    epsilon = 0.2
    for obj in objs:
        if (obj.x - x)**2 + (obj.y - y)**2 <= \
                (obj.collision_radius + collision_radius + epsilon)**2:
            return False
    return True

def check_folder_or_create(folder):
    """ Checks if the folder exits, if not it creates it """
    if not os.path.exists(folder):
        print("Creating folder %s" % folder)
        os.makedirs(folder)

if __name__ == '__main__':
    # get params: number of images, number of envs and folder
    parser = argparse.ArgumentParser(
        description="Arguments to generate a dataset using this environment")

    parser.add_argument('-f', '--destination', type=str, default="dataset_generated",
                        help="Folder destination for the dataset files")
    parser.add_argument('-i', '--num_total_imgs', type=int, default=100,
                        help="Number of images to generate in total")
    parser.add_argument('-e', '--num_total_envs', type=int, default=10,
                        help="Number of environments to generate")
    parser.add_argument('--num_eval_envs', type=int, default=2,
                        help="Number of environments used in evaluation")
    parser.add_argument('--width', type=int, default=300,
                        help="Width in pixels of the images")
    parser.add_argument('--height', type=int, default=300,
                        help="Width in pixels of the images")

    args = parser.parse_args()

    generate_dataset(args.destination, args.num_total_imgs, args.num_total_envs, args.num_eval_envs, width=args.width, height=args.height)
