// Runs the simulated motor at the controller rate (2 kHz) with a fixed current

#include "joint/encoder.hpp"
#include "joint/motor_simulation.hpp"

#include <cstdio>
#include <vector>

constexpr double control_sample_frequency_Hz = 2000.0;
constexpr double control_sample_time_s = 1.0 / control_sample_frequency_Hz;  // 0.5 ms control period
constexpr double simulation_duration_s = 2.0;             // seconds to simulate

// One row of the log file.
struct LogRow {
    double t;
    double current_ref_A;  // target current
    double current_A;      // actual current
    double actual_velocity_rad_per_s;      // true velocity from the simulator
    double encoder_velocity_rad_per_s;   // velocity calculated from encoder counts (what a controller sees)
};

int main() {
    joint::MotorSim motor;
    const uint32_t encoder_resolution_tcks = motor.params().get_encoder_resolution_tcks();
    const int nr_simulation_steps = static_cast<int>(simulation_duration_s * control_sample_frequency_Hz);

    // Reserve all log memory up front, so the loop itself never allocates.
    std::vector<LogRow> log;
    log.reserve(nr_simulation_steps);

    uint32_t actual_position_1n_tcks = motor.get_singleturn_position_tcks();

    for (int k = 0; k < nr_simulation_steps; ++k) {
        const double time_s = k * control_sample_time_s;

        const uint32_t actual_position_tcks = motor.get_singleturn_position_tcks();
        const int32_t position_difference_tcks = joint::position_difference_calculation_tcks(actual_position_tcks, actual_position_1n_tcks, encoder_resolution_tcks);
        const double encoder_velocity_rad_per_s = joint::tcks_to_rad(position_difference_tcks, encoder_resolution_tcks) / control_sample_time_s;
        actual_position_1n_tcks = actual_position_tcks;


        motor.set_target_current(current_cmd);

        const double actual_velocity_rad_per_s = motor.state().velocity_rad_s;  // true velocity at time t
        motor.step(control_sample_time_s);

        log.push_back({time_s, current_cmd, motor.state().actual_current_A, actual_velocity_rad_per_s, encoder_velocity_rad_per_s});
    }

    // Writing the file happens after the loop: file I/O is slow and not real-time.
    std::FILE* f = std::fopen("build/motor_log.csv", "w");
    if (!f) {
        std::printf("could not open build/motor_log.csv\n");
        return 1;
    }
    std::fprintf(f, "t,current_ref_A,current_A,actual_velocity_rad_per_s,encoder_velocity_rad_per_s\n");
    for (const LogRow& r : log) {
        std::fprintf(f, "%.6f,%.4f,%.4f,%.6f,%.6f\n",
                     r.t, r.current_ref_A, r.current_A, r.actual_velocity_rad_per_s, r.encoder_velocity_rad_per_s);
    }
    std::fclose(f);

    std::printf("control %.0f Hz, plant %.0f Hz, %d steps\n",
                control_sample_frequency_Hz, motor.get_plant_sample_frequency_Hz(), nr_simulation_steps);
    std::printf("final angle = %.4f rad (%.2f turns), final speed = %.4f rad/s\n",
                motor.state().angle_rad, motor.state().angle_rad / joint::Two_Pi,
                motor.state().velocity_rad_s);
    std::printf("log written to build/motor_log.csv\n");
    return 0;
}
