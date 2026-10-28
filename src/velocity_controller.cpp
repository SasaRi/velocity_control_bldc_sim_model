#include "joint/velocity_controller.hpp"

#include "joint/encoder.hpp"

#include <cmath>

namespace joint {

VelocityController::VelocityController(double kp_A_per_rad_s, double ki_A_per_rad, double max_current_A,
                                       double control_sample_time_s, uint32_t encoder_resolution_tcks)
    : kp_A_per_rad_s_(kp_A_per_rad_s),
      ki_A_per_rad_(ki_A_per_rad),
      max_current_A_(max_current_A),
      control_sample_time_s_(control_sample_time_s),
      encoder_resolution_tcks_(encoder_resolution_tcks) {
}

void VelocityController::reset(uint32_t position_tcks) {
    previous_position_tcks_ = position_tcks;
    velocity_estimate_rad_s_ = 0.0;
    integral_A_ = 0.0;
}

// Velocity = position change during the last control period / period time.
// This is the average velocity over the last 0.5 ms (about 0.25 ms delay).
// One encoder count per period is 0.096 rad/s.
double VelocityController::estimate_velocity(uint32_t position_tcks) {
    const int32_t position_difference_tcks =
        position_difference_calculation_tcks(position_tcks, previous_position_tcks_, encoder_resolution_tcks_);
    previous_position_tcks_ = position_tcks;
    return tcks_to_rad(position_difference_tcks, encoder_resolution_tcks_) / control_sample_time_s_;
}

double VelocityController::step(double velocity_ref_rad_s, uint32_t position_tcks) {
    velocity_estimate_rad_s_ = estimate_velocity(position_tcks);

    const double error_rad_s = velocity_ref_rad_s - velocity_estimate_rad_s_;
    const double proportional_A = kp_A_per_rad_s_ * error_rad_s;
    const double integral_next_A = integral_A_ + ki_A_per_rad_ * error_rad_s * control_sample_time_s_;

    double current_cmd_A = proportional_A + integral_next_A;

    // Output limit with anti-windup (conditional integration):
    // while the output is limited, the integrator is only updated if the error
    // would take it back out of the limit. Otherwise it would keep growing
    // ("wind up") and cause a large overshoot later.
    if (current_cmd_A > max_current_A_) {
        current_cmd_A = max_current_A_;
        if (error_rad_s < 0.0) integral_A_ = integral_next_A;
    } else if (current_cmd_A < -max_current_A_) {
        current_cmd_A = -max_current_A_;
        if (error_rad_s > 0.0) integral_A_ = integral_next_A;
    } else {
        integral_A_ = integral_next_A;
    }

    // Safety: a NaN or infinite value (e.g. a bad gain) must never reach the drive.
    if (!std::isfinite(current_cmd_A)) {
        integral_A_ = 0.0;
        current_cmd_A = 0.0;
    }

    return current_cmd_A;
}

} // namespace joint
