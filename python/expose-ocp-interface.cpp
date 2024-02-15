#include "qrw/IOCPAbstract.hpp"

#include "qrw/bindings/python.hpp"
#include <eigenpy/optional.hpp>

namespace qrw {

struct OCPWrapper : IOCPAbstract, bp::wrapper<IOCPAbstract> {
  using IOCPAbstract::IOCPAbstract;

  void push_node(uint k, const ConstVecRefN &x0, Matrix34 footsteps, Motion base_vel_ref) override {
    bp::override fn = get_override("push_node");
    fn(k, x0, footsteps, base_vel_ref);
  }

  void solve(std::size_t k) override {
    bp::override fn = get_override("solve");
    fn(k);
  }

  bp::tuple get_results(boost::optional<uint> window_size = boost::none) {
    return get_override("get_results")(window_size);
  }
};

void exposeSolverInterface() {
  eigenpy::OptionalConverter<uint>::registration();
  bp::class_<OCPWrapper, boost::noncopyable>("IOCPAbstract", "Base OCP interface.", bp::no_init)
      .def(bp::init<Params const &>(bp::args("self", "params")))
      .def("solve", bp::pure_virtual(&OCPWrapper::solve), bp::args("self", "k"))
      .def("push_node",
           bp::pure_virtual(&OCPWrapper::push_node),
           bp::args("self", "k", "x0", "footsteps", "base_vel_ref"),
           "Push a new node to the OCP.")
      .def("get_results",
           bp::pure_virtual(&OCPWrapper::get_results),
           (bp::arg("self"), bp::arg("window_size") = boost::none),
           "Fetch the results of the latest MPC iteration.")
      .def_readonly("params", &IOCPAbstract::params_)
      .def_readwrite("num_iters", &IOCPAbstract::num_iters_)
      .def_readonly("max_iter", &IOCPAbstract::max_iter)
      .def_readonly("init_max_iters", &IOCPAbstract::init_max_iters)
      .def_readwrite("xs_init", &IOCPAbstract::xs_init)
      .def_readwrite("us_init", &IOCPAbstract::us_init)
      .def("cycle_warm_start", &IOCPAbstract::cycle_warm_start, bp::args("self"), "Cycle the warm start.")
      .def("warm_start_empty", &IOCPAbstract::warm_start_empty, bp::args("self"), "Check is the warm-start is empty.")
      .def("_check_ws_dim",
           &IOCPAbstract::_check_ws_dim,
           bp::args("self"),
           "Check whether the warm-start has the right size.");
}

}  // namespace qrw
