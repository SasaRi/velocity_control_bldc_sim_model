// Tests of the velocity controller, no framework. Exit code != 0 on failure.

#include "joint/encoder.hpp"
#include "joint/velocity_controller.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok) ++g_failures;
}

constexpr double kTs = 0.0005;
constexpr uint32_t kCounts = 131072;

} // namespace

int main() {
    {   // Output limit: a large error gives exactly +-max current.
        joint::VelocityController c(0.15, 30.0, 10.0, kTs, kCounts);
        c.reset(0);
        check(c.step(1000.0, 0) == 10.0, "large positive error limits to +10 A");
        c.reset(0);
        check(c.step(-1000.0, 0) == -10.0, "large negative error limits to -10 A");
    }

    {   // Anti-windup: while saturated, the integrator does not grow.
        joint::VelocityController c(0.15, 30.0, 10.0, kTs, kCounts);
        c.reset(0);
        for (int k = 0; k < 1000; ++k) c.step(1000.0, 0);  // 0.5 s stuck at the limit
        check(c.get_integral_A() == 0.0, "integrator frozen while output is saturated");
        // Error changes sign: the output must leave the limit immediately.
        check(c.step(-10.0, 0) < 0.0, "output reverses at once when the error reverses");
    }

    {   // Integrator removes a constant error when not saturated.
        joint::VelocityController c(0.15, 30.0, 10.0, kTs, kCounts);
        c.reset(0);
        const double first = c.step(1.0, 0);
        double last = first;
        for (int k = 0; k < 100; ++k) last = c.step(1.0, 0);
        check(last > first, "integrator grows with a constant small error");
    }

    {   // Velocity estimate across the encoder wrap (131071 -> 0).
        joint::VelocityController c(0.0, 0.0, 10.0, kTs, kCounts);
        c.reset(kCounts - 100);
        c.step(0.0, 100);  // moved 200 counts forward through the wrap
        const double expected = joint::tcks_to_rad(200, kCounts) / kTs;  // ~19.2 rad/s
        check(std::abs(c.get_velocity_estimate_rad_s() - expected) < 1e-9, "velocity estimate across wrap");
    }

    {   // Safety: a NaN gain must not reach the drive.
        joint::VelocityController c(std::numeric_limits<double>::quiet_NaN(), 30.0, 10.0, kTs, kCounts);
        c.reset(0);
        check(c.step(100.0, 0) == 0.0, "NaN output is replaced by 0 A");
    }

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASSED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
