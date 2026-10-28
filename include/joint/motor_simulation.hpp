#pragma once

#include "joint/motor_params.hpp"

#include <cstdint>

namespace joint {

// Simulated motor + load.
//
//   current command --> [ideal current loop] --> torque = Kt * i
//   torque          --> [J dw/dt = T - b w - Tc sgn(w)] --> velocity --> angle
//   angle           --> [17-bit encoder] --> raw counts

class MotorSim {
public:
    // Actual (physical) state of the motor
    struct State {
        double angle_rad = 0.0;      // true angle, NOT wrapped (keeps counting up)
        double velocity_rad_s = 0.0; // true angular velocity
        double actual_current_A = 0.0;      // actual current (after the +-peak limit)
        double actual_torque_Nm = 0.0;      // actual motor torque = Kt * actual_current_A
    };

    explicit MotorSim(const MotorParams& params = motorparams, double plant_sample_frequency_Hz = 100e3);

    // Returns the singleturn positon, 0 .. 131071
    uint32_t get_singleturn_position_tcks() const;

    void set_target_current(double current_ref_A);

    void step(double control_sample_time_s);

    // Extra torque from the load
    void set_torque_load(double torque_Nm) { external_torque_Nm_ = torque_Nm; }

    // Resets the motor at a given angle and speed, zero current.
    void reset(double angle_rad = 0.0, double velocity_rad_s = 0.0);

    const State& state() const { return state_; }
    const MotorParams& params() const { return params_; }
    double get_plant_sample_frequency_Hz() const { return plant_sample_frequency_Hz_; }
    double currentRef_A() const { return current_ref_A_; }

private:
    void current_loop_update();

    MotorParams params_;
    double plant_sample_frequency_Hz_;
    double external_torque_Nm_ = 0.0;
    double current_ref_A_ = 0.0;
    State state_;
};

} // namespace joint
