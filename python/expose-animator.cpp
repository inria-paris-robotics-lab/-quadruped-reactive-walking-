#include "qrw/Animator.hpp"
#ifdef QRW_JOYSTICK_SUPPORT
#include "qrw/Joystick.hpp"
#endif

#include "qrw/bindings/visitors.hpp"

namespace qrw {
void exposeKeyboard();

void exposeAnimators() {
  bp::class_<AnimatorBase>(
      "AnimatorBase", "Base animator class. Handles animation using polynomial interpolation.", bp::no_init)
      .def(AnimatorVisitor<AnimatorBase>());

#ifdef QRW_JOYSTICK_SUPPORT
  bp::class_<Joystick, bp::bases<AnimatorBase>>(
      "Joystick", "Animator using an external joystick peripheral.", bp::no_init)
      .def(bp::init<const Params&>(bp::args("self", "params")))
      .def("get_joystick_code", &Joystick::getJoystickCode, bp::args("self"), "Get Joystick Code")
      .def("get_start", &Joystick::getStart, bp::args("self"), "Get Joystick Start")
      .def("get_stop", &Joystick::getStop, bp::args("self"), "Get Joystick Stop")
      .def("get_cross", &Joystick::getCross, bp::args("self"), "Get Joystick Cross status")
      .def("get_circle", &Joystick::getCircle, bp::args("self"), "Get Joystick Circle status")
      .def("get_triangle", &Joystick::getTriangle, bp::args("self"), "Get Joystick Triangle status")
      .def("get_square", &Joystick::getSquare, bp::args("self"), "Get Joystick Square status")
      .def("get_l1", &Joystick::getL1, bp::args("self"), "Get Joystick L1 status")
      .def("get_r1", &Joystick::getR1, bp::args("self"), "Get Joystick R1 status");
#endif
}

}  // namespace qrw
