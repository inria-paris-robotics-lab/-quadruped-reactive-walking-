#include "qrw/Params.hpp"

#include "qrw/bindings/python.hpp"

namespace qrw {

struct params_pickle_suite : bp::pickle_suite {
  static bp::tuple getinitargs(Params const &) { return bp::tuple(); }

  static bp::tuple getstate(bp::object obj) {
    const Params &p = bp::extract<Params const &>(obj)();
    return bp::make_tuple(p.raw_str);
  }

  static void setstate(bp::object obj, bp::tuple state) {
    Params &p = bp::extract<Params &>(obj)();
    auto str = bp::extract<std::string>(state[0])();
    p.initialize_from_str(str);
  }
};

constexpr auto rvp_by_value = bp::return_value_policy<bp::return_by_value>();

void exposeParams() {
  bp::enum_<InterpolationType>("InterpolationType").value("INTERP_CUBIC", InterpolationType::CUBIC).export_values();

  bp::class_<Params>("Params", bp::init<>("self"))
      .def("create_from_file",
           &Params::create_from_file,
           (bp::arg("file_path") = WALK_PARAMETERS_YAML),
           "Create Params from Python with yaml file.\n")
      .staticmethod("create_from_file")
      .def("create_from_str",
           &Params::create_from_str,
           bp::arg("content"),
           "Create Params from Python with a yaml string.\n")
      .staticmethod("create_from_str")
      .def("initialize_from_file",
           &Params::initialize_from_file,
           bp::args("self", "file_path"),
           "Initialize Params from Python with yaml file.\n")
      .def("initialize_from_str",
           &Params::initialize_from_file,
           bp::args("self", "content"),
           "Initialize Params from Python with a yaml string.\n")

      .def_pickle(qrw::params_pickle_suite())
      // Read Params from Python
      .def_readonly("raw_str", &Params::raw_str)
      .def_readonly("config_file", &Params::config_file)
      .def_readonly("interface", &Params::interface)
      .def_readonly("DEMONSTRATION", &Params::DEMONSTRATION)
      .def_readonly("SIMULATION", &Params::SIMULATION)
      .def_readonly("LOGGING", &Params::LOGGING)
      .def_readonly("PLOTTING", &Params::PLOTTING)
      .def_readonly("dt_wbc", &Params::dt_wbc)
      .def_readonly("env_id", &Params::env_id)
      .def_readonly("q_init", &Params::q_init)
      .def_readonly("pose_init", &Params::pose_init)
      .def_readonly("dt_mpc", &Params::dt_mpc)
      .def_readonly("mpc_wbc_ratio", &Params::mpc_wbc_ratio)
      .def_readonly("N_periods", &Params::N_periods)
      .def_readonly("N_SIMULATION", &Params::N_SIMULATION)
      .def_readonly("use_flat_plane", &Params::use_flat_plane)
      .def_readonly("predefined_vel", &Params::predefined_vel)
      .def_readonly("save_guess", &Params::save_guess)
      .def_readonly("movement", &Params::movement)
      .def_readonly("interpolate_mpc", &Params::interpolate_mpc)
      .def_readonly("interpolation_type", &Params::interpolation_type)
      .def_readonly("closed_loop", &Params::closed_loop)
      .def_readonly("kf_enabled", &Params::kf_enabled)
      .def_readonly("Kp_main", &Params::Kp_main)
      .def_readonly("Kd_main", &Params::Kd_main)
      .def_readonly("Kff_main", &Params::Kff_main)
      .def_readonly("starting_nodes", &Params::starting_nodes)
      .def_readonly("ending_nodes", &Params::ending_nodes)
      .def_readonly("gait_repetitions", &Params::gait_repetitions)
      .def_readonly("gait", &Params::get_gait)
      .def_readonly("t_switch", &Params::get_t_switch)
      .def_readonly("v_switch", &Params::get_v_switch)
      .def("set_v_switch", &Params::set_v_switch, bp::args("self", "v_switch"), "Set v_switch matrix from Python.\n")
      .def_readonly("enable_pyb_GUI", &Params::enable_pyb_GUI)
      .def_readonly("asynchronous_mpc", &Params::asynchronous_mpc)
      .def_readonly("mpc_in_rosnode", &Params::mpc_in_rosnode)
      .def_readonly("perfect_estimator", &Params::perfect_estimator)
      .def_readonly("use_qualisys", &Params::use_qualisys)
      .def_readonly("ocp", &Params::ocp)
      .def_readonly("T_gait", &Params::T_gait)
      .def_readonly("N_gait", &Params::N_gait)
      .def_readonly("window_size", &Params::window_size)
      .add_property("task", bp::make_getter(&Params::task, rvp_by_value))
      .add_property("sim", bp::make_getter(&Params::sim, rvp_by_value));

  bp::class_<OCPParams>("OCPParams", bp::no_init)
      .def_readonly("num_threads", &OCPParams::num_threads)
      .def_readonly("max_iter", &OCPParams::max_iter)
      .def_readonly("init_max_iters", &OCPParams::init_max_iters)
      .def_readonly("verbose", &OCPParams::verbose)
      .def_readonly("tol", &OCPParams::tol)
      .def(bp::self_ns::str(bp::self));
}

}  // namespace qrw
