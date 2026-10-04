"""Generate a YOLO dataset (banana / poop) from the CoppeliaSim scene.

The robot is teleported to random poses, the Kinect RGB image is captured exactly
as kinect_node publishes it, and every visible object is labelled by projecting
its 3D bounding box into the image (we know where each object is in simulation).

Usage (CoppeliaSim open with pega_banana_potential_field.ttt, ZMQ on PORT):
    python generate_dataset.py --port 23010 --maps 12 --poses 70 --out dataset
"""
import argparse
import math
import random
import time
from pathlib import Path

import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

CLASSES = {'Banana': 0, 'poop': 1}
MIN_BOX_PX = 4          # ignore objects smaller than this in the image
CLEARANCE = 0.35        # never place the robot this close to an object (it would collect it)

parser = argparse.ArgumentParser()
parser.add_argument('--port', type=int, default=23000)
parser.add_argument('--maps', type=int, default=12)
parser.add_argument('--poses', type=int, default=70)
parser.add_argument('--out', default='dataset')
parser.add_argument('--preview', type=int, default=0, help='save N images with boxes drawn')
args = parser.parse_args()

out = Path(args.out)
client = RemoteAPIClient(port=args.port)
sim = client.require('sim')


def stop_sim():
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.1)


def local_vertices(h):
    """Mesh vertices in the shape frame (the poop mesh is not centred on its origin)."""
    v = np.array(sim.getShapeMesh(h)[0]).reshape(-1, 3)
    return np.hstack([v, np.ones((len(v), 1))])


def capture_rgb(cam):
    sim.handleVisionSensor(cam)                       # render now, with the current pose
    data, res = sim.getVisionSensorImg(cam)
    data = sim.transformImage(data, res, 4)           # same as kinect_node
    img = np.frombuffer(data, dtype=np.uint8).reshape(res[1], res[0], 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)        # same as yolo node


def project(cam, corners_world, W, H, f):
    """World points -> pixel coords. Vision sensor looks along +Z (calibrated: u and v both decrease with +x, +y)."""
    m = np.array(sim.getObjectMatrix(cam, -1)).reshape(3, 4)
    R, t = m[:, :3], m[:, 3]
    pc = (corners_world[:, :3] - t) @ R               # world -> camera frame
    if np.any(pc[:, 2] < 0.05):
        return None
    u = W / 2 - f * pc[:, 0] / pc[:, 2]
    v = H / 2 - f * pc[:, 1] / pc[:, 2]
    return u, v


for split in ('train', 'val'):
    (out / 'images' / split).mkdir(parents=True, exist_ok=True)
    (out / 'labels' / split).mkdir(parents=True, exist_ok=True)

robot = sim.getObject('/myRobot')
cam = sim.getObject('/myRobot/kinect/rgb')
motors = [sim.getObject('/myRobot/leftMotor'), sim.getObject('/myRobot/rightMotor')]
W, H = sim.getVisionSensorResolution(cam)
f = (max(W, H) / 2) / math.tan(sim.getObjectFloatParam(cam, sim.visionfloatparam_perspective_angle) / 2)
z0 = sim.getObjectPosition(robot, -1)[2]

count, n_boxes = 0, {0: 0, 1: 0}
for m in range(args.maps):
    stop_sim()
    time.sleep(1.1)                                   # buildScene seeds with os.time()
    sim.startSimulation()
    time.sleep(1.0)
    sim.setBoolProperty(cam, 'explicitHandling', True)
    for motor in motors:                              # the saved scene has non-zero targets
        sim.setJointTargetVelocity(motor, 0)
    objs = []
    for h in sim.getObjectsInTree(sim.handle_scene):
        a = sim.getObjectAlias(h)
        if a in CLASSES:
            objs.append((h, CLASSES[a], local_vertices(h)))
    split = 'val' if m == args.maps - 1 else 'train'

    for k in range(args.poses):
        poses = [(np.array(sim.getObjectMatrix(h, -1)).reshape(3, 4), c, lc) for h, c, lc in objs]
        poses = [p for p in poses if p[0][2, 3] < 100]  # skip collected objects
        xy = np.array([[p[0][0, 3], p[0][1, 3]] for p in poses])
        for _ in range(100):
            if random.random() < 0.7 and len(poses):
                # look at a random object from 0.4..2.5 m
                tx, ty = xy[random.randrange(len(xy))]
                d, a = random.uniform(0.4, 2.5), random.uniform(-math.pi, math.pi)
                x, y = tx + d * math.cos(a), ty + d * math.sin(a)
                yaw = math.atan2(ty - y, tx - x) + random.uniform(-0.45, 0.45)
            else:
                x, y = random.uniform(-2.7, 2.7), random.uniform(-2.7, 2.7)
                yaw = random.uniform(-math.pi, math.pi)
            if len(xy) == 0 or np.min(np.hypot(xy[:, 0] - x, xy[:, 1] - y)) > CLEARANCE:
                break
        sim.setObjectPosition(robot, -1, [x, y, z0])
        sim.setObjectOrientation(robot, -1, [0, 0, yaw - math.pi / 2])  # front is local +Y
        sim.resetDynamicObject(robot)
        img = capture_rgb(cam)

        lines, boxes = [], []
        for M, c, lc in poses:
            cw = lc @ np.vstack([M, [0, 0, 0, 1]]).T
            uv = project(cam, cw, W, H, f)
            if uv is None:
                continue
            u, v = uv
            x1, x2 = max(u.min(), 0), min(u.max(), W - 1)
            y1, y2 = max(v.min(), 0), min(v.max(), H - 1)
            full = (u.max() - u.min()) * (v.max() - v.min())
            if x2 - x1 < MIN_BOX_PX or y2 - y1 < MIN_BOX_PX or (x2 - x1) * (y2 - y1) < 0.4 * full:
                continue
            dist = math.hypot(M[0, 3] - x, M[1, 3] - y)
            if dist > 3.5:                            # beyond the Kinect range
                continue
            lines.append(f'{c} {(x1 + x2) / 2 / W:.6f} {(y1 + y2) / 2 / H:.6f} {(x2 - x1) / W:.6f} {(y2 - y1) / H:.6f}')
            boxes.append((c, int(x1), int(y1), int(x2), int(y2)))
            n_boxes[c] += 1

        name = f'm{m:02d}_{k:03d}'
        cv2.imwrite(str(out / 'images' / split / f'{name}.jpg'), img)
        (out / 'labels' / split / f'{name}.txt').write_text('\n'.join(lines))
        if count < args.preview:
            dbg = img.copy()
            for c, a, b, cc, d in boxes:
                cv2.rectangle(dbg, (a, b), (cc, d), (0, 255, 255) if c == 0 else (0, 0, 255), 1)
            cv2.imwrite(str(out / f'preview_{count:02d}.png'), dbg)
        count += 1
    print(f'map {m + 1}/{args.maps}: {count} images, boxes banana={n_boxes[0]} poop={n_boxes[1]}', flush=True)

sim.setBoolProperty(cam, 'explicitHandling', False)
stop_sim()
(out / 'data.yaml').write_text(
    f'path: {out.resolve()}\ntrain: images/train\nval: images/val\nnames:\n  0: banana\n  1: poop\n')
print('done', count, n_boxes)
