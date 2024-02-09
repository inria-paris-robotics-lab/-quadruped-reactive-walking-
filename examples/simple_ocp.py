import numpy as np
import pinocchio as pin
import quadruped_reactive_walking as qrw
from quadruped_reactive_walking.controller import Controller
import time
import pprint
import aligator

import matplotlib.pyplot as plt

from quadruped_reactive_walking.wb_mpc import AlgtrOCPProx, CrocOCP, OCP_TYPE_MAP
from quadruped_reactive_walking.wb_mpc.task_spec import TaskSpec

print("OCP registered types:", pprint.pformat(OCP_TYPE_MAP), sep="\n")
params = qrw.Params.create_from_file()
params.ocp: qrw.OCPParams
task = TaskSpec(params)

print(task)

base_vel_refs = [pin.Motion(np.zeros(6)) for _ in range(params.N_gait)]

x0 = task.x0
print("x0:", x0)

ocp = CrocOCP(params, base_vel_refs)

nsteps = ocp.ddp.problem.T
xs_i = [x0] * (nsteps + 1)
us_i = ocp.ddp.problem.quasiStatic(xs_i[:nsteps])

ocp.xs_init = xs_i
ocp.us_init = us_i
ocp.solve(0)

print("============== PARALLEL ===================")

ocp2 = AlgtrOCPProx(params, base_vel_refs, aligator.LinearSolverChoice.LQ_SOLVER_SERIAL)

ts = time.time()
ocp2.solve(0)

ocp2_res: aligator.Results = ocp2.solver.results

dist_x = np.linalg.norm(np.stack(ocp.ddp.xs) - np.stack(ocp2_res.xs))
print("Dist X (par):", dist_x)

ctrler = Controller(params, params.q_init, CrocOCP)

print("============== SERIAL ===================")

ocp3 = AlgtrOCPProx(params, base_vel_refs, aligator.LinearSolverChoice.LQ_SOLVER_SERIAL)

ts = time.time()
ocp3.solve(0)

ocp3_res: aligator.Results = ocp3.solver.results

dist_x = np.linalg.norm(np.stack(ocp.ddp.xs) - np.stack(ocp3_res.xs))
print("Dist X (ser):", dist_x)
dist_x = np.linalg.norm(np.stack(ocp2_res.xs) - np.stack(ocp3_res.xs))
dist_u = np.linalg.norm(np.stack(ocp2_res.us) - np.stack(ocp3_res.us))
print("DIST X (SER-PAR):", dist_x)

np.set_printoptions(precision=2, linewidth=250)

for i in [0, 1, 2, 3, 4, 5, 6, 10, 20]:
    print("==== i = {} ====".format(i))
    K0_par = ocp2_res.controlFeedbacks()[0]
    K0_ser = ocp3_res.controlFeedbacks()[0]
    print("K0_par=")
    print(K0_par)
    print("K0_ser=")
    print(K0_ser)
    K0_err = np.abs(K0_par - K0_ser)
    print("K0 ERROR=")
    print(K0_err)
    print("|K ERR|=\n", np.linalg.norm(K0_err, np.inf))
    plt.figure()
    plt.title("FB matrix err $t={}$".format(i))
    plt.imshow(np.log(K0_err))
    plt.colorbar()

plt.show()
