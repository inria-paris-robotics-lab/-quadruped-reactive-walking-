#include "qrw/MPCResult.hpp"

#include "qrw/bindings/python.hpp"
#include "qrw/bindings/custom-members.hpp"

namespace qrw {

void exposeMPCResult() {
  bp::class_<MPCResult>(
      "MPCResult",
      "MPC result struct.",
      bp::init<uint, uint, uint, uint, bp::optional<uint>>(bp::args("self", "Ngait", "nx", "nu", "ndx", "window_size")))
      .def_readwrite("gait", &MPCResult::gait)
      .add_property("xs",
                    bp::make_getter(&MPCResult::xs),
                    make_non_resizable_vec_member(&MPCResult::xs),
                    "Predicted future trajectory.")
      .add_property("us",
                    bp::make_getter(&MPCResult::us),
                    make_non_resizable_vec_member(&MPCResult::us),
                    "Predicted feedforward controls.")
      .add_property("K",
                    bp::make_getter(&MPCResult::Ks),
                    make_non_resizable_vec_member(&MPCResult::Ks),
                    "Feedback gains for the controller.")
      .def_readwrite("solving_duration", &MPCResult::solving_duration)
      .def_readonly(
          "window_size", &MPCResult::get_window_size, "Size of the window to pass back to the MPC controller.")
      .def_readwrite("num_iters", &MPCResult::num_iters)
      .def_readwrite("new_result", &MPCResult::new_result);
}

}  // namespace qrw
