#include "qrw/bindings/python.hpp"

#include "qrw/Animator.hpp"

namespace qrw {

template <typename Class>
struct AnimatorVisitor : bp::def_visitor<AnimatorVisitor<Class>> {
  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<Params&>(bp::args("self", "params"), "Default constructor."))

        .def("update_v_ref", &Class::update_v_ref, bp::args("self", "k", "gait_is_static"), "Update joystick values.")
        .def_readonly("p_ref", &Class::p_ref_, "Get Reference Position")
        .def_readonly("v_ref", &Class::v_ref_, "Get Reference Velocity");
  }
};

}  // namespace qrw
