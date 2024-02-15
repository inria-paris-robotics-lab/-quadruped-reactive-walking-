import numpy as np
import pinocchio as pin

from pinocchio import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer


def make_meshcat_viz(robot: RobotWrapper, meshColor=(0.6, 0.1, 0.1, 0.8)) -> MeshcatVisualizer:
    import hppfcl

    plane = hppfcl.Plane(np.array([0, 0, 1]), 0.0)
    geobj = pin.GeometryObject("ground", 0, pin.SE3.Identity(), plane)
    geobj.meshColor[:] = meshColor

    vmodel = robot.visual_model
    vmodel.addGeometryObject(geobj)

    vizer = MeshcatVisualizer(robot.model, robot.collision_model, vmodel, data=robot.data)
    vizer.initViewer(loadModel=True)
    vizer.setBackgroundColor()
    return vizer


class MeshcatViewer:
    def __init__(self, robot: RobotWrapper):
        self.rmodel = robot.model
        self.rdata = robot.data
        meshColor = 0.7, 1.0, 0.65, 1.0

        self.viz = make_meshcat_viz(robot, meshColor)

    def update(self, controller, device):
        q = np.zeros(19)
        q[7:] = device.joints.positions
        q[:7] = pin.SE3ToXYZQUAT(device.oMb)
        q[2] = device.baseState[2]
        self.viz.display(q)
