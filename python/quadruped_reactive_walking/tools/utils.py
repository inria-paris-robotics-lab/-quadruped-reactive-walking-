import numpy as np
import pinocchio as pin
import copy

try:
    from multiprocess.shared_memory import SharedMemory
except ImportError:
    from multiprocessing.shared_memory import SharedMemory

def quaternionToRPY(quat):
    """
    Quaternion (4 x 0) to Roll Pitch Yaw (3 x 1)
    """
    quat = pin.Quaternion(quat)
    return pin.utils.matrixToRpy(quat.matrix())


def no_copy_roll_insert(x, a):
    """
    No copy left roll along the 0-th axis, inserting element `a` at the end.
    Works on lists and numpy arrays.
    """
    tail = x[1:]
    x[:-1] = tail
    x[-1] = a


def no_copy_roll(x):
    """No copy (except for the head) left roll along the 0-th axis."""
    tmp = copy.copy(x[0])
    no_copy_roll_insert(x, tmp)


def create_shared_ndarray(shape, dtype, shm: SharedMemory):
    """
    Create a ndarray using a shared memory buffer, using another array's shape and dtype.

    DO NOT call this with an unbound SharedMemory object, i.e.
    >>> create_shared_ndarray_from_other(a, SharedMemory(*args))
    The shared memory object will be garbage collected.
    """
    return np.ndarray(shape, dtype, buffer=shm.buf)
