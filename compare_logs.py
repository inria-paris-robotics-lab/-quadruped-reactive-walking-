import numpy as np
from pathlib import Path
import quadruped_reactive_walking as qrw
import argparse
import crocoddyl
import example_robot_data as erd
import matplotlib.pyplot as plt
from matplotlib import colors

robot = erd.load("solo12")
rmodel = robot.model
state = crocoddyl.StateMultibody(rmodel)

params = qrw.Params.create_from_file()
WBC_RATIO = params.mpc_wbc_ratio
print(WBC_RATIO)

TMPDIR = Path.home() / ".tmp"
print("TMPDIR =", TMPDIR)

ALLLOGS = list(TMPDIR.glob("logs/2023_*"))
print(ALLLOGS)

parser = argparse.ArgumentParser()
parser.add_argument("log1", type=Path)
parser.add_argument("log2", type=Path)
args = parser.parse_args()
print(args)

assert args.log1 in ALLLOGS
assert args.log2 in ALLLOGS

file1 = open(args.log1 / "data.npz", "rb")
file2 = open(args.log2 / "data.npz", "rb")

data1 = np.load(file1)
data2 = np.load(file2)


def get_solver_name(data):
    return data["solver_cls"].item()


KEYS = list(data1.keys())

assert "ocp_xs" in KEYS
assert "ocp_us" in KEYS

plt.rcParams["lines.linewidth"] = 1.0
plt.rcParams["figure.dpi"] = 120

xs1 = data1["ocp_xs"]
us1 = data1["ocp_us"]

NITER = us1.shape[0]
NSTEPS = us1.shape[1]

xs2 = data2["ocp_xs"]
us2 = data2["ocp_us"]

NJOINTS = 12
fbs1 = data1["ocp_K"]
fbs2 = data2["ocp_K"]

assert fbs1.shape[1] == NJOINTS
assert fbs1.shape[2] == state.ndx


# Plotting
def infNorm(x, **kwargs):
    return np.linalg.norm(x, ord=np.inf, **kwargs)


Xerr = np.zeros((NITER, NSTEPS + 1, state.ndx))
for i in range(NITER):
    for j in range(NSTEPS + 1):
        Xerr[i, j, :] = state.diff(xs1[i, j], xs2[i, j])
Uerr = us2 - us1

Xerr_over_mpc = infNorm(Xerr[::WBC_RATIO], axis=(1, 2))
Uerr_over_mpc = infNorm(Uerr[::WBC_RATIO], axis=(1, 2))

K_err = fbs2 - fbs1
K_err = K_err[::WBC_RATIO]
K_err_over_mpc = infNorm(K_err, axis=(1, 2))

plt.figure()
plt.plot(Xerr_over_mpc, label="State err")
plt.plot(Uerr_over_mpc, label="Control err", ls="--")
plt.plot(K_err_over_mpc, label="Feedback err")
plt.xlabel("MPC cycle")
plt.ylabel("Error")
plt.yscale("log")
plt.legend()
plt.grid(which="both")
plt.title("Comparison between {:s} and {:s}".format(get_solver_name(data1), get_solver_name(data2)))

# Error over time & mpc iteration
Xerr_over_time = infNorm(Xerr[::WBC_RATIO], axis=2)

aspect = NSTEPS / (NITER // WBC_RATIO)
plt.figure()
im = plt.imshow(Xerr_over_time, cmap="viridis", norm=colors.LogNorm(), aspect=aspect)
plt.colorbar()
plt.ylabel("MPC cycle")
plt.xlabel("Horizon $t$")
plt.title("state error")

Uerr_over_time = infNorm(Uerr[::WBC_RATIO], axis=2)

plt.figure()
im = plt.imshow(Uerr_over_time, cmap="viridis", norm=colors.LogNorm(), aspect=aspect)
plt.colorbar()
plt.ylabel("MPC cycle")
plt.xlabel("Horizon $t$")
plt.title("ctrl error")

# Error over first MPC cycles
MPC_PLOT_IDX = np.arange(0, 90, WBC_RATIO)
Xerr_over_1st_loop = Xerr[MPC_PLOT_IDX, :, :]
Xerr_over_1st_loop = infNorm(Xerr_over_1st_loop, axis=2)
plt.figure()
lines = plt.plot(Xerr_over_1st_loop.T)
plt.legend(lines, range(len(MPC_PLOT_IDX)), title="MPC cycle")
plt.xlabel("Horizon $t$")
plt.ylabel("Err")
plt.yscale("log")
plt.grid(which="both")
plt.title("Error on MPC output - state $x^*$")


Uerr_over_1st_loop = Uerr[MPC_PLOT_IDX, :, :]
Uerr_over_1st_loop = np.linalg.norm(Uerr_over_1st_loop, axis=2)
plt.figure()
lines = plt.plot(Uerr_over_1st_loop.T)
plt.legend(lines, range(len(MPC_PLOT_IDX)), title="MPC cycle")
plt.xlabel("Horizon $t$")
plt.yscale("log")
plt.grid(which="both")
plt.title("Error on MPC output - control $u^*$")


_, axes = plt.subplots(3, 4, sharex=True, figsize=(10, 7))
for i, ax in enumerate(axes.flat):
    plt.sca(ax)
    plt.plot(us1[WBC_RATIO * 4, :, i], alpha=0.6, label="log1")
    plt.plot(us2[WBC_RATIO * 4, :, i], alpha=0.6, label="log2", ls="--")
    plt.legend()

plt.tight_layout()


# plot tau ff
WBC_PLOT_MAX = WBC_RATIO * 30
tauff1 = data1["wbc_tau_ff"]
tauff2 = data2["wbc_tau_ff"]
plt.figure()
plt.subplot(121)
plt.plot(infNorm(tauff1 - tauff2, axis=1)[:WBC_PLOT_MAX])
plt.xlabel("WBC iter")
plt.yscale("log")
plt.title("Err. on WBC feedforward torque $\\tau$")
plt.grid(which="both")


def get_wbc_x_des(data):
    wbc_q_des = data["wbc_q_des"]
    wbc_v_des = data["wbc_v_des"]
    wbc_x_des = np.concatenate([wbc_q_des, wbc_v_des], axis=1)
    return wbc_x_des


wbc_x_des1 = get_wbc_x_des(data1)
wbc_x_des2 = get_wbc_x_des(data2)
wbc_x_err = infNorm(wbc_x_des1 - wbc_x_des2, axis=1)
plt.subplot(122)
plt.plot(wbc_x_err[:WBC_PLOT_MAX])
plt.title("Err. on WBC target state $\\hat{x}$")
plt.yscale("log")
plt.grid(which="both")


def get_x_estim(data):
    qfil = data["q_filtered"]
    vfil = data["v_filtered"]
    x_estim = np.concatenate([qfil, vfil], axis=1)
    return x_estim


x_estim_1 = get_x_estim(data1)
x_estim_2 = get_x_estim(data2)
x_e_err = np.array([state.diff(u, v) for u, v in zip(x_estim_1, x_estim_2)])
x_e_err = infNorm(x_e_err[:WBC_PLOT_MAX], axis=1)
plt.figure()
plt.plot(x_e_err)
plt.title("Estimated state $x_\\mathrm{estim}$ error")
plt.xlabel("WBC iter.")
plt.yscale("log")
plt.grid(which="both")

FIGDIR = Path("figs")
FIGDIR.mkdir(exist_ok=True)
figs = [plt.figure(n) for n in plt.get_fignums()]
labels = [
    "errors_per_iter",
    "imshow_state",
    "imshow_control",
    "mpc_err_traj_state",
    "mpc_err_traj_control",
    "control_traj_iter0",
    "wbc_tau_ff_state_err",
    "x_estim_err",
]
for fig, lab in zip(figs, labels):
    fpath = (FIGDIR / lab).with_suffix(".png")
    print(fpath)
    fig.savefig(fpath)
