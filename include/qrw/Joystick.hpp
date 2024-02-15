///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for Joystick class
///
/// \details This class handles computations related to the reference velocity
/// of the robot
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOYSTICK_H_INCLUDED
#define JOYSTICK_H_INCLUDED

#include <chrono>
#include <linux/joystick.h>

#include "qrw/Animator.hpp"

struct gamepad_status {
  double v_x = 0.0;    // Up/down status of left pad
  double v_y = 0.0;    // Left/right status of left pad
  double v_z = 0.0;    // Up/down status of right pad
  double w_yaw = 0.0;  // Left/right status of right pad
  int start = 0;       // Status of Start button
  int select = 0;      // Status of Select button
  int cross = 0;       // Status of cross button
  int circle = 0;      // Status of circle button
  int triangle = 0;    // Status of triangle button
  int square = 0;      // Status of square button
  int L1 = 0;          // Status of L1 button
  int R1 = 0;          // Status of R1 button
};

class Joystick : public AnimatorBase {
 public:
  /// \brief Constructor
  ///
  /// \param[in] params Object that stores parameters
  Joystick(Params const& params);

  /// \brief Destructor.
  ~Joystick() override;

  /// \brief Update the status of the joystick by reading the status of the
  /// gamepad
  /// \param[in] k Numero of the current loop
  /// \param[in] gait_is_static If the Gait is in or is switching to a static
  /// gait
  void update_v_ref(int k, bool gait_is_static) override;

  /// \brief Check if a gamepad event occured and read its data
  /// \param[in] fd Identifier of the gamepad object
  /// \param[in] event Gamepad event object
  int read_event(int fd, struct js_event* event);

  int getJoystickCode() { return joystick_code_; }
  bool getStop() { return stop_; }
  bool getStart() { return start_; }
  bool getCross() { return gamepad.cross == 1; }
  bool getCircle() { return gamepad.circle == 1; }
  bool getTriangle() { return gamepad.triangle == 1; }
  bool getSquare() { return gamepad.square == 1; }
  bool getL1() { return gamepad.L1 == 1; }
  bool getR1() { return gamepad.R1 == 1; }

 private:
  Vector6 p_gp_;                // Raw position reference of the gamepad
  Vector6 v_gp_;                // Raw velocity reference of the gamepad
  Vector6 v_ref_heavy_filter_;  // Reference velocity after heavy low pass filter

  int joystick_code_ = 0;  // Code to trigger gait changes
  bool stop_ = false;      // Flag to stop the controller
  bool start_ = false;     // Flag to start the controller

  // How much the gamepad velocity and position is filtered to avoid sharp
  // changes
  double gp_alpha_vel = 0.0;                 // Low pass filter coefficient for v_ref_ (if gamepad-controlled)
  double gp_alpha_pos = 0.0;                 // Low pass filter coefficient for p_ref_
  double gp_alpha_vel_heavy_filter = 0.002;  // Low pass filter coefficient for v_ref_heavy_filter_

  // Maximum velocity values
  double vXScale = 0.3;    // Lateral
  double vYScale = 0.5;    // Forward
  double vYawScale = 1.0;  // Rotation

  // Maximum position values
  double pRollScale = -0.32;  // Lateral
  double pPitchScale = 0.32;  // Forward
  double pHeightScale = 0.0;  // Raise/Lower the base
  double pYawScale = -0.35;   // Rotation Yaw

  // Variable to handle the automatic static/trot switching
  bool switch_static = false;   // Flag to switch to a static gait
  bool lock_gp = true;          // Flag to lock the output velocity when we are switching back to trot gait
  double lock_duration_ = 1.0;  // Duration of the lock in seconds

  std::chrono::time_point<std::chrono::system_clock> lock_time_static_;  // Timestamp of the start of the lock
  std::chrono::time_point<std::chrono::system_clock> lock_time_L1_;      // Timestamp of the latest L1 pressing

  // Gamepad client variables
  gamepad_status gamepad;  // Structure that stores gamepad status
  const char* device;      // Gamepad device object
  int js;                  // Identifier of the gamepad object
  struct js_event event;   // Gamepad event object
};

#endif  // JOYSTICK_H_INCLUDED
