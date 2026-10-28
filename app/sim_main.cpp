// Test scenario: the PI velocity controller (2 kHz) drives the simulated motor.
//
//   t = 10 ms   velocity command steps from 0 to 200 rad/s
//   t = 100 ms  0.5 Nm load torque against the motion
//   t = 200 ms  end
//
// Runs as fast as possible; it does not wait for real time.
// Writes build/velocity_log.csv and prints a summary.

#include "joint/motor_simulation.hpp"
#include "joint/velocity_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

constexpr double control_sample_frequency_Hz = 2000.0;
constexpr double control_sample_time_s = 1.0 / control_sample_frequency_Hz;  // 0.5 ms

// Test scenario
constexpr double velocity_step_time_s = 0.010;
constexpr double velocity_step_rad_s = 200.0;
constexpr double load_step_time_s = 0.100;
constexpr double load_torque_Nm = -0.5;  // negative: against the (positive) motion
constexpr double simulation_duration_s = 0.200;

// Controller gains (see README, "Tuning")
constexpr double kp_A_per_rad_s = 0.15;
constexpr double ki_A_per_rad = 30.0;

// One row of the log file.
struct LogRow {
    double time_s;
    double velocity_cmd_rad_s;   // velocity command
    double velocity_est_rad_s;   // estimated from the encoder (what the controller sees)
    double velocity_true_rad_s;  // true velocity from the simulator
    double current_cmd_A;        // current command from the controller
    double load_torque_Nm;       // external load torque
};

// Summary values required by the assignment, calculated from the true velocity.
void print_summary(const std::vector<LogRow>& log) {
    const double v_10 = 0.1 * velocity_step_rad_s;
    const double v_90 = 0.9 * velocity_step_rad_s;
    const double band_rad_s = 0.02 * velocity_step_rad_s;  // +-2 % band

    double t_10 = -1.0, t_90 = -1.0;
    double max_before_load = 0.0;
    double error_sum = 0.0;
    int error_count = 0;
    double min_after_load = velocity_step_rad_s;
    double last_outside_band_s = load_step_time_s;
    double peak_current_A = 0.0;

    for (const LogRow& r : log) {
        const double v = r.velocity_true_rad_s;
        peak_current_A = std::max(peak_current_A, std::abs(r.current_cmd_A));

        if (r.time_s < load_step_time_s) {
            if (t_10 < 0.0 && v >= v_10) t_10 = r.time_s;
            if (t_90 < 0.0 && v >= v_90) t_90 = r.time_s;
            max_before_load = std::max(max_before_load, v);
            // Steady state: the last 20 ms before the load step
            if (r.time_s >= load_step_time_s - 0.020) {
                error_sum += r.velocity_cmd_rad_s - v;
                ++error_count;
            }
        } else {
            min_after_load = std::min(min_after_load, v);
            if (std::abs(v - r.velocity_cmd_rad_s) > band_rad_s) last_outside_band_s = r.time_s;
        }
    }

    const double overshoot_pct = (max_before_load - velocity_step_rad_s) / velocity_step_rad_s * 100.0;
    const double steady_state_error = error_sum / error_count;
    const double dip_rad_s = velocity_step_rad_s - min_after_load;
    const double recovery_s = last_outside_band_s + control_sample_time_s - load_step_time_s;

    std::printf("\n--- Summary (true velocity) ---\n");
    std::printf("rise time 10-90 %%             : %.2f ms\n", (t_90 - t_10) * 1e3);
    std::printf("overshoot                     : %.2f %% (%.2f rad/s)\n", overshoot_pct, max_before_load - velocity_step_rad_s);
    std::printf("steady-state error (80-100 ms): %.3f rad/s\n", steady_state_error);
    std::printf("velocity dip after load       : %.2f rad/s (min %.2f rad/s)\n", dip_rad_s, min_after_load);
    std::printf("recovery time (within 2 %%)    : %.2f ms\n", recovery_s * 1e3);
    std::printf("peak current command          : %.2f A\n", peak_current_A);
}

int main() {
    joint::MotorSim motor;
    joint::VelocityController controller(kp_A_per_rad_s, ki_A_per_rad, motor.params().max_current_A,
                                         control_sample_time_s, motor.params().get_encoder_resolution_tcks());
    controller.reset(motor.get_singleturn_position_tcks());

    const int nr_simulation_steps = static_cast<int>(std::lround(simulation_duration_s * control_sample_frequency_Hz));

    // Reserve all log memory up front, so the loop itself never allocates.
    std::vector<LogRow> log;
    log.reserve(static_cast<size_t>(nr_simulation_steps));

    for (int k = 0; k < nr_simulation_steps; ++k) {
        const double time_s = k * control_sample_time_s;

        // Test scenario inputs
        const double velocity_cmd_rad_s = (time_s >= velocity_step_time_s) ? velocity_step_rad_s : 0.0;
        const double load_Nm = (time_s >= load_step_time_s) ? load_torque_Nm : 0.0;
        motor.set_torque_load(load_Nm);

        // Controller: reads only the encoder, writes the current command
        const double current_cmd_A = controller.step(velocity_cmd_rad_s, motor.get_singleturn_position_tcks());
        motor.set_target_current(current_cmd_A);

        // Physics: one control period passes
        const double velocity_true_rad_s = motor.state().velocity_rad_s;  // true velocity at time_s
        motor.step(control_sample_time_s);

        log.push_back({time_s, velocity_cmd_rad_s, controller.get_velocity_estimate_rad_s(),
                       velocity_true_rad_s, current_cmd_A, load_Nm});
    }

    // Writing the file happens after the loop: file I/O is slow and not real-time.
    std::FILE* f = std::fopen("build/velocity_log.csv", "w");
    if (!f) {
        std::printf("could not open build/velocity_log.csv (run from the project folder)\n");
        return 1;
    }
    std::fprintf(f, "time_s,velocity_cmd_rad_s,velocity_est_rad_s,velocity_true_rad_s,current_cmd_A,load_torque_Nm\n");
    for (const LogRow& r : log) {
        std::fprintf(f, "%.4f,%.3f,%.3f,%.3f,%.4f,%.3f\n", r.time_s, r.velocity_cmd_rad_s,
                     r.velocity_est_rad_s, r.velocity_true_rad_s, r.current_cmd_A, r.load_torque_Nm);
    }
    std::fclose(f);

    std::printf("control %.0f Hz, plant %.0f Hz, Kp = %.3f A/(rad/s), Ki = %.1f A/rad\n",
                control_sample_frequency_Hz, motor.get_plant_sample_frequency_Hz(), kp_A_per_rad_s, ki_A_per_rad);
    print_summary(log);
    std::printf("\nlog written to build/velocity_log.csv\n");
    return 0;
}
