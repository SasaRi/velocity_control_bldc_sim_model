// Minimal self-checking tests, no framework. Exit code != 0 on failure.

#include "joint/encoder.hpp"
#include "joint/motor_simulation.hpp"

#include <cmath>
#include <cstdio>

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok) ++g_failures;
}

constexpr double kTs = 1.0 / 2000.0;

void run(joint::MotorSim& sim, double current, double seconds) {
    const int n = static_cast<int>(std::lround(seconds / kTs));
    for (int k = 0; k < n; ++k) {
        sim.set_target_current(current);
        sim.step(kTs);
    }
}

} // namespace

int main() {
    const auto& p = joint::motorparams;
    const double tau = p.J_kgm2 / p.b_Nms_per_rad;  // 2 s

    {   // Below breakaway torque the shaft does not move at all.
        joint::MotorSim sim;
        run(sim, 0.15, 1.0);
        check(sim.state().velocity_rad_s == 0.0 && sim.state().angle_rad == 0.0,
              "stiction: 0.015 Nm < Tc keeps shaft at rest");
    }

    {   // Spin-up matches w(t) = w_ss (1 - exp(-t/tau)), w_ss = (Kt i - Tc) / b.
        joint::MotorSim sim;
        const double i = 0.3;
        run(sim, i, 1.0);
        const double w_ss = (p.Kt_Nm_per_A * i - p.Tc_Nm) / p.b_Nms_per_rad;
        const double w_ref = w_ss * (1.0 - std::exp(-1.0 / tau));
        const double err = std::abs(sim.state().velocity_rad_s - w_ref) / w_ref;
        std::printf("       w = %.4f, analytic = %.4f, rel err = %.2e\n",
                    sim.state().velocity_rad_s, w_ref, err);
        check(err < 1e-4, "spin-up follows analytic first-order response");
    }

    {   // Coast-down stops at t_stop = tau * ln(1 + w0 b / Tc) and stays stopped.
        joint::MotorSim sim;
        const double w0 = 100.0;
        sim.reset(0.0, w0);
        const double t_stop = tau * std::log(1.0 + w0 * p.b_Nms_per_rad / p.Tc_Nm);
        run(sim, 0.0, t_stop - 0.002);
        check(sim.state().velocity_rad_s > 0.0, "coast: still moving just before t_stop");
        run(sim, 0.0, 0.004);
        check(sim.state().velocity_rad_s == 0.0, "coast: stopped just after t_stop");
        run(sim, 0.0, 0.5);
        check(sim.state().velocity_rad_s == 0.0, "coast: no chatter / creep after stop");
    }

    {   // Strong enough reverse torque drives through zero instead of sticking.
        joint::MotorSim sim;
        sim.reset(0.0, 50.0);
        run(sim, -0.3, 0.5);
        check(sim.state().velocity_rad_s < 0.0, "reversal: passes through zero under -0.03 Nm");
    }

    {   // Plant rate: clamped to 20-100 kHz, and the result is converged, i.e.
        // the controller sees the same "continuous" plant at either rate.
        check(joint::MotorSim(p).get_plant_sample_frequency_Hz() == 100e3, "default plant rate is 100 kHz");
        check(joint::MotorSim(p, 1e3).get_plant_sample_frequency_Hz() == 20e3, "plant rate clamps up to 20 kHz");
        check(joint::MotorSim(p, 1e6).get_plant_sample_frequency_Hz() == 100e3, "plant rate clamps down to 100 kHz");

        joint::MotorSim slow(p, 20e3), fast(p, 100e3);
        joint::MotorSim* sims[] = {&slow, &fast};
        for (auto* sim : sims) {
            run(*sim, 0.5, 0.2);   // spin up
            run(*sim, -0.5, 0.3);  // brake through zero and reverse
            run(*sim, 0.0, 0.5);   // coast to a stop
        }
        const double dw = std::abs(slow.state().velocity_rad_s - fast.state().velocity_rad_s);
        const double dth = std::abs(slow.state().angle_rad - fast.state().angle_rad);
        std::printf("       20 vs 100 kHz: |dw| = %.2e rad/s, |dtheta| = %.2e rad\n", dw, dth);
        check(dw < 1e-6 && dth < 1e-3, "20 kHz and 100 kHz plants agree");
    }

    {   // Target vs actual current: the target is stored as given, the actual
        // current follows it only when time advances (step).
        joint::MotorSim sim;
        sim.set_target_current(5.0);
        check(sim.currentRef_A() == 5.0, "target current stored as given");
        check(sim.state().actual_current_A == 0.0, "actual current unchanged before step()");
        sim.step(kTs);
        check(sim.state().actual_current_A == 5.0, "actual current follows target after step()");
        check(sim.state().actual_torque_Nm == p.Kt_Nm_per_A * 5.0, "torque = Kt * current");
    }

    {   // External load torque acts on the shaft.
        joint::MotorSim sim;
        sim.set_torque_load(-0.5);
        run(sim, 5.0, 0.01);  // 0.5 Nm motor torque against 0.5 Nm load
        check(sim.state().velocity_rad_s == 0.0, "load cancels equal motor torque: no motion");
        run(sim, 0.0, 0.01);  // load alone turns the shaft backwards
        check(sim.state().velocity_rad_s < 0.0, "load torque alone drives shaft negative");
    }

    {   // Encoder quantization and wrap.
        const uint32_t counts = p.get_encoder_resolution_tcks();
        joint::MotorSim sim;
        check(counts == 131072u, "17-bit encoder = 131072 counts/rev");
        sim.reset(joint::PI);
        check(sim.get_singleturn_position_tcks() == counts / 2, "angle pi reads half a turn");
        sim.reset(-1e-9);
        check(sim.get_singleturn_position_tcks() == counts - 1, "just below zero wraps to max count");
        sim.reset(3.0 * joint::Two_Pi + joint::PI);
        check(sim.get_singleturn_position_tcks() == counts / 2, "3.5 turns reads the same as half a turn");
        check(joint::position_difference_calculation_tcks(5, counts - 5, counts) == 10, "delta across wrap, forward");
        check(joint::position_difference_calculation_tcks(counts - 5, 5, counts) == -10, "delta across wrap, backward");
    }

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASSED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
