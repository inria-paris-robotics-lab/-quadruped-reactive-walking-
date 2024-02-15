#include "qrw/IMPCWrapper.hpp"
#include "qrw/bindings/python.hpp"

namespace qrw {

template <class MPCDerived = IMPCWrapper>
struct PyMPCWrapper : MPCDerived, bp::wrapper<MPCDerived> {
  using MPCDerived::MPCDerived;
  using StdVecVecN = std::vector<VectorN>;

  void solve(uint k, const ConstVecRefN &x0, Vector4 footstep, Motion base_vel_ref) override {
    bp::override fn = this->get_override("solve");
    fn(k, x0, footstep, base_vel_ref);
  }

  MPCResult get_latest_result() const override { return this->get_override("get_latest_result")(); }
};

void exposeMPCInterface() {
  bp::class_<PyMPCWrapper<>, boost::noncopyable>("IMPCWrapper", bp::no_init)
      .def(bp::init<Params const &>(bp::args("self", "params")))
      .add_property("WINDOW_SIZE", &IMPCWrapper::window_size)
      .def_readonly("params", &IMPCWrapper::params_)
      .add_property("N_gait", &IMPCWrapper::N_gait)
      .def("get_latest_result", bp::pure_virtual(&PyMPCWrapper<>::get_latest_result), bp::args("self"))
      .def("solve",
           bp::pure_virtual(&PyMPCWrapper<>::solve),
           (bp::arg("self"), bp::arg("k"), bp::arg("x0"), bp::arg("footstep"), bp::arg("base_vel_ref")));
}

}  // namespace qrw
