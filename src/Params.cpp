#include "qrw/Params.hpp"
#include <iostream>

template struct YAML::convert<MatrixN>;
template struct YAML::convert<VectorN>;

std::ostream &operator<<(std::ostream &oss, const OCPParams &p) {
  oss << "OCPParams {"
      << "\n\tnum_threads:\t" << p.num_threads << "\n\tmax_iter:\t" << p.max_iter << "\n\tinit_max_iters:\t"
      << p.init_max_iters << "\n\tverbose:\t" << p.verbose << "\n}";
  return oss;
}

int compute_k_mpc(const Params &params) { return static_cast<int>(std::round(params.dt_mpc / params.dt_wbc)); }

Params::Params()
    : raw_str(""),
      config_file(""),
      interface(""),
      DEMONSTRATION(false),
      SIMULATION(false),
      LOGGING(false),
      PLOTTING(false),
      env_id(0),
      use_flat_plane(false),
      predefined_vel(false),
      N_SIMULATION(0),
      enable_pyb_GUI(false),
      perfect_estimator(false),
      use_qualisys(false),
      asynchronous_mpc(false),
      mpc_in_rosnode(false),

      q_init(12),  // Fill with zeros, will be filled with values later
      pose_init(7),
      dt_wbc(0.0),
      dt_mpc(0.0),
      N_periods(0),
      save_guess(false),
      movement(""),
      interpolate_mpc(true),
      interpolation_type(InterpolationType::CUBIC),
      closed_loop(true),
      kf_enabled(false),
      Kp_main(3),
      Kd_main(3),
      Kff_main(0.0),

      starting_nodes(0),
      ending_nodes(0),
      gait_repetitions(0),  // Fill with zeros, will be filled with values later
      gait_vec(),
      gp_alpha_vel(0.0),  // Fill with zeros, will be filled with values later

      gp_alpha_pos(0.0),

      t_switch(),
      v_switch(),
      fc_v_esti(0.0),

      T_gait(0.0),  // Period of the gait
      footsteps_under_shoulders(12)  // Fill with zeros, will be filled with values later
{
  Kp_main.setZero();
  Kd_main.setZero();
  q_init.setZero();
  pose_init.setZero();
  footsteps_under_shoulders.setZero();
}

Params Params::create_from_file(const std::string &file_path) {
  Params params;
  params.initialize_from_file(expand_env(file_path));
  return params;
}

Params Params::create_from_str(const std::string &content) {
  Params params;
  params.initialize_from_str(content);
  return params;
}

void Params::initialize_common(const YAML::Node &node) {
  const YAML::Node &robot_node = node["robot"];
  YAML::convert<Params>::decode(robot_node, *this);
  std::cout << "Loading robot config file " << config_file << std::endl;
  task = node["task"];
  sim = node["sim"];

  mpc_wbc_ratio = (int)(dt_mpc / dt_wbc);

  // Save the raw_str
  YAML::Emitter emitter;
  emitter << node;
  raw_str.assign(emitter.c_str());
}

void Params::initialize_from_file(const std::string &file_path) {
  std::cout << "Loading params file " << file_path << std::endl;
  // Load YAML file
  assert_file_exists(file_path);
  YAML::Node param = YAML::LoadFile(file_path);

  // Check if YAML node is detected and retrieve it
  assert_yaml_parsing(param, file_path, "robot");
  initialize_common(param);
}

void Params::initialize_from_str(const std::string &content) {
  // Load YAML file
  YAML::Node param = YAML::Load(content);

  // Check if YAML node is detected and retrieve it
  assert_yaml_parsing(param, "[yamlstring]", "robot");
  initialize_common(param);
}

namespace YAML {

bool convert<OCPParams>::decode(const Node &node, OCPParams &rhs) {
  rhs.num_threads = node["num_threads"].as<uint>();
  rhs.max_iter = node["max_iter"].as<uint>();
  rhs.init_max_iters = node["init_max_iters"].as<uint>();
  rhs.verbose = node["verbose"].as<bool>();
  rhs.tol = node["tol"].as<double>();
  return true;
}

bool convert<Params>::decode(const Node &robot_node, Params &rhs) {
  // Retrieve robot parameters
  assert_yaml_parsing(robot_node, "robot", "config_file");
  rhs.config_file = expand_env(robot_node["config_file"].as<std::string>());

  assert_yaml_parsing(robot_node, "robot", "interface");
  rhs.interface = robot_node["interface"].as<std::string>();

  assert_yaml_parsing(robot_node, "robot", "DEMONSTRATION");
  rhs.DEMONSTRATION = robot_node["DEMONSTRATION"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "SIMULATION");
  rhs.SIMULATION = robot_node["SIMULATION"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "LOGGING");
  rhs.LOGGING = robot_node["LOGGING"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "PLOTTING");
  rhs.PLOTTING = robot_node["PLOTTING"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "dt_wbc");
  rhs.dt_wbc = robot_node["dt_wbc"].as<Scalar>();

  assert_yaml_parsing(robot_node, "robot", "env_id");
  rhs.env_id = robot_node["env_id"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "q_init");
  YAML::convert<VectorN>::decode(robot_node["q_init"], rhs.q_init);

  assert_yaml_parsing(robot_node, "robot", "pose_init");
  YAML::convert<VectorN>::decode(robot_node["pose_init"], rhs.pose_init);

  assert_yaml_parsing(robot_node, "robot", "window_size");
  rhs.window_size = robot_node["window_size"].as<uint>();

  assert_yaml_parsing(robot_node, "robot", "dt_mpc");
  rhs.dt_mpc = robot_node["dt_mpc"].as<Scalar>();

  assert_yaml_parsing(robot_node, "robot", "N_periods");
  rhs.N_periods = robot_node["N_periods"].as<uint>();

  assert_yaml_parsing(robot_node, "robot", "N_SIMULATION");
  rhs.N_SIMULATION = robot_node["N_SIMULATION"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "use_flat_plane");
  rhs.use_flat_plane = robot_node["use_flat_plane"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "predefined_vel");
  rhs.predefined_vel = robot_node["predefined_vel"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "enable_pyb_GUI");
  rhs.enable_pyb_GUI = robot_node["enable_pyb_GUI"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "asynchronous_mpc");
  rhs.asynchronous_mpc = robot_node["asynchronous_mpc"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "mpc_in_rosnode");
  rhs.mpc_in_rosnode = robot_node["mpc_in_rosnode"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "perfect_estimator");
  rhs.perfect_estimator = robot_node["perfect_estimator"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "use_qualisys");
  rhs.use_qualisys = robot_node["use_qualisys"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "ocp");
  rhs.ocp = robot_node["ocp"].as<OCPParams>();

  assert_yaml_parsing(robot_node, "robot", "save_guess");
  rhs.save_guess = robot_node["save_guess"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "movement");
  rhs.movement = robot_node["movement"].as<std::string>();

  assert_yaml_parsing(robot_node, "robot", "interpolate_mpc");
  rhs.interpolate_mpc = robot_node["interpolate_mpc"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "interpolation_type");
  rhs.interpolation_type = (InterpolationType)robot_node["interpolation_type"].as<uint>();

  assert_yaml_parsing(robot_node, "robot", "closed_loop");
  rhs.closed_loop = robot_node["closed_loop"].as<bool>();

  assert_yaml_parsing(robot_node, "robot", "Kp_main");
  rhs.Kp_main = robot_node["Kp_main"].as<VectorN>();

  assert_yaml_parsing(robot_node, "robot", "Kd_main");
  rhs.Kd_main = robot_node["Kd_main"].as<VectorN>();

  assert_yaml_parsing(robot_node, "robot", "Kff_main");
  rhs.Kff_main = robot_node["Kff_main"].as<Scalar>();

  assert_yaml_parsing(robot_node, "robot", "starting_nodes");
  rhs.starting_nodes = robot_node["starting_nodes"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "ending_nodes");
  rhs.ending_nodes = robot_node["ending_nodes"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "gait_repetitions");
  rhs.gait_repetitions = robot_node["gait_repetitions"].as<int>();

  assert_yaml_parsing(robot_node, "robot", "gait");
  rhs.gait_vec = robot_node["gait"].as<VectorNi>();
  rhs.convert_gait_vec();

  assert_yaml_parsing(robot_node, "robot", "gp_alpha_vel");
  rhs.gp_alpha_vel = robot_node["gp_alpha_vel"].as<Scalar>();

  assert_yaml_parsing(robot_node, "robot", "gp_alpha_pos");
  rhs.gp_alpha_pos = robot_node["gp_alpha_pos"].as<Scalar>();

  assert_yaml_parsing(robot_node, "robot", "t_switch");
  rhs.t_switch = robot_node["t_switch"].as<VectorN>();

  assert_yaml_parsing(robot_node, "robot", "v_switch");
  rhs.v_switch.resize(6, rhs.t_switch.size());
  YAML::convert<RowMatrix6N>::decode(robot_node["v_switch"], rhs.v_switch);

  assert_yaml_parsing(robot_node, "robot", "fc_v_esti");
  rhs.fc_v_esti = robot_node["fc_v_esti"].as<Scalar>();

  assert_yaml_parsing(robot_node, "robot", "solo3D");
  rhs.solo3D = robot_node["solo3D"].as<bool>();

  if (!rhs.SIMULATION) rhs.perfect_estimator = false;
  return true;
}
}  // namespace YAML

void Params::convert_gait_vec() {
  if (gait_vec.size() % 5 != 0) {
    throw std::runtime_error(
        "gait matrix in yaml is not in the correct format. It should have five "
        "columns, with the first column containing the number of timestep for "
        "each phase and the four others containing 0 and 1 to describe the "
        "feet status during that phase.");
  }

  if (N_periods < 1) {
    throw std::runtime_error("N_periods should be larger than 1.");
  }

  // Get the number of lines in the gait matrix
  N_gait = 0;
  for (uint i = 0; i < gait_vec.size() / 5; i++) {
    N_gait += gait_vec[5 * i];
  }

  // Save period of the gait
  T_gait = N_gait * dt_mpc;

  // Resize gait matrix
  gait.resize((Index)N_gait * N_periods, 4);

  // Fill gait matrix
  int k = 0;
  for (uint i = 0; i < gait_vec.size() / 5; i++) {
    for (int j = 0; j < gait_vec[5 * i]; j++) {
      gait.row(k) << gait_vec[5 * i + 1], gait_vec[5 * i + 2], gait_vec[5 * i + 3], gait_vec[5 * i + 4];
      k++;
    }
  }

  // Repeat gait for other periods
  for (uint i = 1; i < N_periods; i++) {
    gait.block(i * (Index)N_gait, 0, N_gait, 4) = gait.block(0, 0, N_gait, 4);
  }
}
