import quadruped_reactive_walking as qrw
from quadruped_reactive_walking.wb_mpc import CrocOCP as OCP
from quadruped_reactive_walking.wb_mpc.task_spec import TaskSpecFull
from quadruped_reactive_walking.wb_mpc.target import Target

import crocoddyl
import sys
import time

T = int(sys.argv[1]) if (len(sys.argv) > 1) else int(5e3)  # number of trials
MAXITER = 1


def createProblem():
    params = qrw.Params.create_from_file()
    pd = TaskSpecFull(params)
    target = Target(params)

    x0 = pd.x0_reduced

    ocp = OCP(target)
    ocp.push_node()
    ocp.x0 = x0

    xs = [x0] * (ocp.ddp.problem.T + 1)
    us = ocp.ddp.problem.quasiStatic([x0] * ocp.ddp.problem.T)
    return xs, us, ocp.problem


def runDDPSolveBenchmark(xs, us, problem):
    ddp = crocoddyl.SolverDDP(problem)

    duration = []
    for _ in range(T):
        c_start = time.time()
        ddp.solve(xs, us, MAXITER, False, 0.1)
        c_end = time.time()
        duration.append(1e3 * (c_end - c_start))

    avrg_duration = sum(duration) / len(duration)
    min_duration = min(duration)
    max_duration = max(duration)
    return avrg_duration, min_duration, max_duration


def runShootingProblemCalcBenchmark(xs, us, problem):
    duration = []
    for _ in range(T):
        c_start = time.time()
        problem.calc(xs, us)
        c_end = time.time()
        duration.append(1e3 * (c_end - c_start))

    avrg_duration = sum(duration) / len(duration)
    min_duration = min(duration)
    max_duration = max(duration)
    return avrg_duration, min_duration, max_duration


def runShootingProblemCalcDiffBenchmark(xs, us, problem):
    duration = []
    for _ in range(T):
        c_start = time.time()
        problem.calcDiff(xs, us)
        c_end = time.time()
        duration.append(1e3 * (c_end - c_start))

    avrg_duration = sum(duration) / len(duration)
    min_duration = min(duration)
    max_duration = max(duration)
    return avrg_duration, min_duration, max_duration


print("\033[1m")
print("Python bindings:")
xs, us, problem = createProblem()
avrg_duration, min_duration, max_duration = runDDPSolveBenchmark(xs, us, problem)
print("  DDP.solve [ms]: {0} ({1}, {2})".format(avrg_duration, min_duration, max_duration))
avrg_duration, min_duration, max_duration = runShootingProblemCalcBenchmark(xs, us, problem)
print("  ShootingProblem.calc [ms]: {0} ({1}, {2})".format(avrg_duration, min_duration, max_duration))
avrg_duration, min_duration, max_duration = runShootingProblemCalcDiffBenchmark(xs, us, problem)
print("  ShootingProblem.calcDiff [ms]: {0} ({1}, {2})".format(avrg_duration, min_duration, max_duration))
print("\033[0m")
