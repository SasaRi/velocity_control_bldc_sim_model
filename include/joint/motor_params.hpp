#pragma once

#include <cstdint>

namespace joint {

inline constexpr double PI    = 3.14159265358979323846;
inline constexpr double Two_Pi = 2.0 * PI;

// Physical motor + load parameters
struct MotorParams {
    double   Kt_Nm_per_A;                       // torque constant             [Nm/A]
    double   max_current_A;                     // current limit               [A]
    double   J_kgm2;                            // rotor + load inertia        [kg m^2]
    double   b_Nms_per_rad;                     // viscous friction b          [Nm s/rad]
    double   Tc_Nm;                             // Coulomb (dry) friction Tc   [Nm]
    uint32_t encoder_nr_singleturn_bits;        // single-turn absolute encoder resolution

    constexpr uint32_t get_encoder_resolution_tcks() const { return (uint32_t)1 << encoder_nr_singleturn_bits; }
    constexpr double   get_maxTorque_Nm() const { return Kt_Nm_per_A * max_current_A; }
};

inline constexpr MotorParams motorparams{
    0.1,    // Kt
    10.0,   // peak current
    2e-5,   // J
    1e-5,   // b
    0.02,   // Tc
    17,     // 17-bit single-turn
};

} // namespace joint
