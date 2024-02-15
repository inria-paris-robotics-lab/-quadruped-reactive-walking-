import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from concurrent.futures import ThreadPoolExecutor


def numpy_to_multiarray_float64(np_array):
    multiarray = Float64MultiArray()
    multiarray.layout.dim = [
        MultiArrayDimension("dim%d" % i, np_array.shape[i], np_array.shape[i] * np_array.dtype.itemsize)
        for i in range(np_array.ndim)
    ]
    multiarray.data = np_array.ravel().tolist()
    return multiarray


def multiarray_to_numpy_float64(ros_array):
    dims = [d.size for d in ros_array.layout.dim]
    if dims == []:
        return np.array([])
    out = np.empty(dims)
    out.ravel()[:] = ros_array.data
    return out


def listof_numpy_to_multiarray_float64(list):
    return numpy_to_multiarray_float64(np.array(list))


def multiarray_to_listof_numpy_float64(ros_array):
    return list(multiarray_to_numpy_float64(ros_array))


class AsyncServiceProxy(object):
    """Asynchronous ROS service proxy
    Example 1:
        add_two_ints_async = AsyncServiceProxy('add_two_ints',AddTwoInts)
        fut = add_two_ints_async(1, 2)
        while not fut.done():
            print('Waiting...')
        try:
            print('Result: {}'.format(fut.result()))
        except ServiceException:
            print('Service failed!')

    Example 2:
        def result_cb(fut):
            try:
                print('Result: {}'.format(fut.result()))
            except ServiceException:
                print('Service failed!')
        add_two_ints_async = AsyncServiceProxy('add_two_ints',AddTwoInts,callback=result_cb)
        fut = add_two_ints_async(1, 2)
        while not fut.done():
            print('Waiting...')
    """

    def __init__(self, service_name, service_type, persistent=True, headers=None, callback=None):
        """Create an asynchronous service proxy."""

        self.executor = ThreadPoolExecutor(max_workers=1)
        self.service_proxy = rospy.ServiceProxy(service_name, service_type, persistent, headers)
        self.callback = callback

    def __call__(self, *args, **kwargs):
        """Get a Future corresponding to a call of this service."""

        fut = self.executor.submit(self.service_proxy.call, *args, **kwargs)
        if self.callback is not None:
            fut.add_done_callback(self.callback)

        return fut

    def __del__(self):
        self.executor.shutdown()
