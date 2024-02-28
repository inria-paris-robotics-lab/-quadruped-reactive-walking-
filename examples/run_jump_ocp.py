import pprint
import crocoddyl  # noqa
import aligator
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt

from quadruped_reactive_walking import Params
from quadruped_reactive_walking.ocp_defs import jump
from crocoddyl import ShootingProblem
from quadruped_reactive_walking.tools.kinematics_utils import get_translation_array
from quadruped_reactive_walking.tools.meshcat_viewer import make_meshcat_viz
from aligator.croc import convertCrocoddylProblem


params = Params.create_from_file()
base_vel_refs = [pin.Motion(np.zeros(6)) for _ in range(params.N_gait)]

ocp_spec = jump.JumpOCPBuilder(params, base_vel_refs)
robot = ocp_spec.task.robot
rmodel = robot.model

pprint.pprint(ocp_spec.jump_spec)

problem: ShootingProblem = ocp_spec.problem
nsteps = params.N_gait
dt = params.dt_mpc
x0 = ocp_spec.x0
xs_init = [x0 for _ in range(params.N_gait + 1)]
us_init = problem.quasiStatic(xs_init[:nsteps])


# solver = crocoddyl.SolverFDDP(problem)
# solver.setCallbacks([crocoddyl.CallbackVerbose()])
# solver.solve(xs_init, us_init)
# xs = solver.xs
# us = solver.us

problem_cv = convertCrocoddylProblem(problem)
solver = aligator.SolverProxDDP(1e-3, 1e-5, verbose=aligator.VERBOSE)
solver.setup(problem_cv)
solver.run(problem_cv, xs_init, us_init)
xs = solver.results.xs
us = solver.results.us

qs = [x[: rmodel.nq] for x in xs]

ROOT_JOINT_ID = rmodel.getFrameId("base_link")
frame_pos, frame_vels = get_translation_array(rmodel, xs, ROOT_JOINT_ID)

times = np.linspace(0.0, nsteps * dt, nsteps + 1)

labs = ["x", "y", "z"]
for i in range(3):
    plt.subplot(1, 3, i + 1)
    plt.plot(times, frame_pos[:, i])
    plt.ylabel("${}$".format(labs[i]))
plt.tight_layout()
plt.show()


vizer = make_meshcat_viz(robot)
vizer.viewer.open()

while True:
    input()
    vizer.play(qs, dt)
    print("[play again?]", end="")
