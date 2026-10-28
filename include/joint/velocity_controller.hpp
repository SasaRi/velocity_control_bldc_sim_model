#pragma once

#include <cstdint>

namespace joint {

// Velocity loop of the joint drive, called every control period (2 kHz).
//
//   encoder position --> [velocity estimate] --> velocity
//   velocity reference - velocity --> [PI + limit + anti-windup] --> current command
//
// The controller only sees the encoder position, never the true velocity.
// step() is written as if it ran in an interrupt: no memory allocation,
// no exceptions, no blocking.
class VelocityController {
public:
    VelocityController(double kp_A_per_rad_s, double ki_A_per_rad, double max_current_A,
                       double control_sample_time_s, uint32_t encoder_resolution_tcks);

    // Start from a known encoder position with an empty integrator.
    void reset(uint32_t position_tcks);

    // One control period: returns the current command [A], limited to +-max_current_A.
    double step(double velocity_ref_rad_s, uint32_t position_tcks);

    double get_velocity_estimate_rad_s() const { return velocity_estimate_rad_s_; }
    double get_integral_A() const { return integral_A_; }

private:
    double estimate_velocity(uint32_t position_tcks);

    // Settings
    double kp_A_per_rad_s_;
    double ki_A_per_rad_;
    double max_current_A_;
    double control_sample_time_s_;
    uint32_t encoder_resolution_tcks_;

    // State
    uint32_t previous_position_tcks_ = 0;
    double velocity_estimate_rad_s_ = 0.0;
    double integral_A_ = 0.0;  // I-part of the PI output
};

} // namespace joint
