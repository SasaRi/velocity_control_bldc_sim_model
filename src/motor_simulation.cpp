#include "joint/motor_simulation.hpp"

#include <algorithm>
#include <cmath>

namespace joint {

// Sign of x: +1, -1 or 0.
static double sign(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

MotorSim::MotorSim(const MotorParams& params, double plant_sample_frequency_Hz)
    : params_(params),
      plant_sample_frequency_Hz_(std::clamp(plant_sample_frequency_Hz, 20e3, 100e3)) {
}

void MotorSim::reset(double angle_rad, double velocity_rad_s) {
    state_ = State{};
    state_.angle_rad = angle_rad;
    state_.velocity_rad_s = velocity_rad_s;
    external_torque_Nm_ = 0.0;
    current_ref_A_ = 0.0;
}

// Set the target current
void MotorSim::set_target_current(double current_ref_A) {
    current_ref_A_ = current_ref_A;
}

// Update the current and torque feedback
void MotorSim::current_loop_update() {
    // Ideal current loop: the actual current equals the reference instantaneously.
    // The current limit is applied in the velocity controller.
    state_.actual_current_A = current_ref_A_;
    state_.actual_torque_Nm = params_.Kt_Nm_per_A * state_.actual_current_A;
}

// Update the joint mechanical plant
// Mechanics:  J * dw/dt = T_motor + T_external - b*w - Tc*sign(w)
void MotorSim::step(double control_sample_time_s) {

    current_loop_update();

    // Number of sub-steps, rounded to the nearest whole number (at least 1).
    const int n = std::max(1, static_cast<int>(std::lround(control_sample_time_s * plant_sample_frequency_Hz_)));
    const double plant_sample_time_s = control_sample_time_s / n;
    for (int k = 0; k < n; ++k) {
        // Add load torque to the motor torque
        const double torque_applied_Nm = state_.actual_torque_Nm + external_torque_Nm_;

        // Check if the torque can overcome the dry friction, If not, stay put
        const bool too_weak_to_move = std::abs(torque_applied_Nm) <= params_.Tc_Nm;
        if (state_.velocity_rad_s == 0.0 && too_weak_to_move) {
            continue;
        }

        // Dry friction opposes the motion (or, at rest, the push that starts it)
        const double friction_direction =
            (state_.velocity_rad_s != 0.0) ? sign(state_.velocity_rad_s) : sign(torque_applied_Nm);

        // Calculate the resulting joint torque
        const double torque_joint_Nm =
            torque_applied_Nm - params_.b_Nms_per_rad * state_.velocity_rad_s - params_.Tc_Nm * friction_direction;

         // acceleration = torque / inertia
        double velocity_next_rad_s =
            state_.velocity_rad_s + (torque_joint_Nm / params_.J_kgm2) * plant_sample_time_s;

        // Friction slowed us through zero in this step: stop instead of reversing.
        if (state_.velocity_rad_s * velocity_next_rad_s < 0.0 && too_weak_to_move) {
            velocity_next_rad_s = 0.0;
        }

        // Angle is the integral of velocity (average of old and new speed).
        state_.angle_rad += 0.5 * (state_.velocity_rad_s + velocity_next_rad_s) * plant_sample_time_s;
        state_.velocity_rad_s = velocity_next_rad_s;
    }
}

// Calculate the singleturn position from the rotor angle
uint32_t MotorSim::get_singleturn_position_tcks() const {
    const double singleturn_resolution_tcks = params_.get_encoder_resolution_tcks();      // 131072
    const double turns = state_.angle_rad / Two_Pi;             // e.g. 3.25 turns
    const double fraction = turns - std::floor(turns);          // 0.25 (also works < 0)
    const auto wrapped_actual_position_tcks = static_cast<uint32_t>(fraction * singleturn_resolution_tcks);
    return wrapped_actual_position_tcks % params_.get_encoder_resolution_tcks();                       // guard the 1.0 edge
}

} // namespace joint
