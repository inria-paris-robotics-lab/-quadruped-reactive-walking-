///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for Params class
///
/// \details This class retrieves and stores all parameters written in the main
/// .yaml so that the user can easily change their value without digging into
/// the code. It also stores some model parameters whose values depends on what
/// is in the yaml
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PARAMS_H_INCLUDED
#define PARAMS_H_INCLUDED

#include "qrw/yaml-eigen.hpp"

#include <fstream>
#include <vector>
#include <cstdlib>
#include <string>
#include <regex>

#include "qrw/Types.hpp"
#include "qrw/utils.hpp"

#ifndef WALK_PARAMETERS_YAML
#error Variable WALK_PARAMETERS_YAML not defined.
#endif

extern template struct YAML::convert<MatrixN>;
extern template struct YAML::convert<VectorN>;

// fwd-declaration
struct Params;

int compute_k_mpc(const Params &params);

enum InterpolationType : uint { CUBIC = 3 };

struct OCPParams {
  uint num_threads;
  uint max_iter;
  uint init_max_iters;
  bool verbose;
  double tol;
};

std::ostream &operator<<(std::ostream &oss, const OCPParams &p);

namespace YAML {
template <>
struct convert<Params> {
  static bool decode(const Node &robot_node, Params &rhs);
};

template <>
struct convert<OCPParams> {
  static bool decode(const Node &node, OCPParams &rhs);
};

}  // namespace YAML

struct Params {
  /// \brief Constructor using a path to a configuration file.
  Params();

  static Params create_from_file(const std::string &file_path = WALK_PARAMETERS_YAML);
  static Params create_from_str(const std::string &content);

  /// \brief Destructor.
  ~Params() = default;

  /// \brief Initializer
  /// \param[in] file_path File path to the yaml file
  void initialize_from_file(const std::string &file_path);

  /// \brief Initializer
  /// \param[in] content Content of the yaml.
  void initialize_from_str(const std::string &content);

  /// \brief Convert the gait vector of the yaml into an Eigen matrix
  void convert_gait_vec();

  Eigen::Ref<MatrixNi> get_gait() { return gait; }
  Eigen::Ref<VectorN> get_t_switch() { return t_switch; }
  Eigen::Ref<RowMatrix6N> get_v_switch() { return v_switch; }
  void set_v_switch(Eigen::Ref<const RowMatrix6N> &v_switch_in) { v_switch = v_switch_in; }

  std::string raw_str;

  // See .yaml file for meaning of parameters
  // General parameters
  std::string config_file;  // Name of the yaml file containing hardware information
  std::string interface;    // Name of the communication interface (check with ifconfig)
  bool DEMONSTRATION;       // Enable/disable demonstration functionalities
  bool SIMULATION;          // Enable/disable PyBullet simulation or running on real robot
  bool LOGGING;             // Enable/disable logging during the experiment
  bool PLOTTING;            // Enable/disable automatic plotting at the end of the experiment
  int env_id;               // Identifier of the environment to choose in which one the simulation will happen
  bool use_flat_plane;      // If True the ground is flat, otherwise it has bumps
  bool predefined_vel;      // If we are using a predefined reference velocity (True) or a joystick (False)
  int N_SIMULATION;         // Number of simulated wbc time steps
  bool enable_pyb_GUI;      // Enable/disable PyBullet GUI
  bool perfect_estimator;   // Enable/disable perfect estimator by using data directly from PyBullet
  bool use_qualisys;        // Enable/disable mocap
  OCPParams ocp;            // OCP parameters

  bool asynchronous_mpc;  // Run the MPC in an asynchronous process parallel of the main loop
  bool mpc_in_rosnode;    // Run the MPC on a separate rosnode

  // General control parameters
  VectorN q_init;     // Initial articular positions
  VectorN pose_init;  // Initial base pose
  Scalar dt_wbc;      // Time step of the whole body control
  Scalar dt_mpc;      // Time step of the model predictive control
  int mpc_wbc_ratio;
  uint N_periods;                        // Number of gait periods in the MPC prediction horizon
  bool save_guess;                       // true to save the initial result of the mpc
  bool verbose;                          // verbosity
  std::string movement;                  // Name of the mmovemnet to perform
  bool interpolate_mpc;                  // true to interpolate the impedance quantities, otherwise integrate
  InterpolationType interpolation_type;  // type of interpolation used
  bool closed_loop;                      // true to close the MPC loop
  bool kf_enabled;                       // Use complementary filter (False) or kalman filter (True) for the estimator
  VectorN Kp_main;                       // Proportional gains for the PD+
  VectorN Kd_main;                       // Derivative gains for the PD+
  Scalar Kff_main;                       // Feedforward torques multiplier for the PD+

  // Parameters of Gait
  int starting_nodes;
  int ending_nodes;
  int gait_repetitions;  // number of times the gait is used in the whole walk cycle
  VectorNi gait_vec;     // Initial gait matrix (vector)

  // Parameters of Joystick
  Scalar gp_alpha_vel;   //  Coefficient of the low pass filter applied to gamepad velocity
  Scalar gp_alpha_pos;   //  Coefficient of the low pass filter applied to gamepad position
  VectorN t_switch;      // Predefined velocity switch times matrix
  RowMatrix6N v_switch;  // Predefined velocity switch values matrix

  // Parameters of Estimator
  Scalar fc_v_esti;  // Cut frequency for the low pass that filters the  estimated base velocity

  bool solo3D;  // Enable the 3D environment with corresponding planner blocks

  // Not defined in yaml
  MatrixNi gait;     // Initial gait matrix (Eigen)
  Scalar T_gait;     // Period of the gait
  int N_gait;        // Number of steps in gait
  uint window_size;  // Window size

  YAML::Node task;
  YAML::Node sim;

 private:
  void initialize_common(const YAML::Node &robot_node);
};

/// \brief Check if a parameter exists in a given yaml file (bofore we try
/// retrieving its value)
///
/// \param[in] yaml_node Name of the yaml file
/// \param[in] parent_node_name Name of the parent node
/// \param[in] child_node_name Name of the child node
#define assert_yaml_parsing(yaml_node, parent_node_name, child_node_name)                 \
  if (!yaml_node[child_node_name]) {                                                      \
    std::ostringstream oss;                                                               \
    oss << "Error: Wrong parsing of the YAML file from src file: [" << __FILE__ << "]"    \
        << ", in function: [" << __FUNCTION__ << "]"                                      \
        << ", line: " << __LINE__ << ". Node [" << child_node_name << "] does not exists" \
        << "under the node [" << parent_node_name << "].";                                \
    throw std::runtime_error(oss.str());                                                  \
  }                                                                                       \
  assert(true)

/// \brief Check if a file exists (before we try loading it)
///
/// \param[in] filename File path to check
#define assert_file_exists(filename)                               \
  std::ifstream f(filename.c_str());                               \
  if (!f.good()) {                                                 \
    std::ostringstream oss;                                        \
    oss << "Error: Problem opening the file [" << filename << "]"  \
        << ", from src file: [" << __FILE__ << "]"                 \
        << ", in function: [" << __FUNCTION__ << "]"               \
        << ", line: [" << __LINE__ << ". The file may not exist."; \
    throw std::runtime_error(oss.str());                           \
  }                                                                \
  assert(true)

#endif  // PARAMS_H_INCLUDED
