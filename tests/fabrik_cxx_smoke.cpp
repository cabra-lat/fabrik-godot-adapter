#include "fabrik_core.h"

#include <cmath>
#include <cstdint>
#include <iostream>

int main() {
    float joints[] = {0, 0, 0, 1, 0, 0, 2, 0, 0};
    float lengths[] = {1, 1};
    const float target[] = {1, 1, 0};
    float residual = 0;
    const int32_t status = fabrik_solve_f32(joints, 3, lengths, target, 1,
        0.00001f, 64, joints, &residual);
    if (status != FABRIK_OK || !std::isfinite(residual) || residual > 0.00001f) {
        std::cerr << "reachable solve failed: status=" << status
                  << " residual=" << residual << "\n";
        return 1;
    }

    float measured[] = {0, 0, 0, 1, 0, 0, 2, 0, 0};
    const int32_t measured_status = fabrik_solve_f32(measured, 3, nullptr, target, 1,
        0.00001f, 64, measured, &residual);
    if (measured_status != FABRIK_OK) {
        std::cerr << "measured-length solve failed: status=" << measured_status << "\n";
        return 1;
    }

    float unreachable[] = {0, 0, 0, 1, 0, 0, 2, 0, 0};
    const float distant[] = {0, 4, 0};
    const int32_t unreachable_status = fabrik_solve_f32(unreachable, 3, lengths,
        distant, 1, 0.00001f, 64, unreachable, &residual);
    if (unreachable_status != FABRIK_UNREACHABLE || residual <= 0.00001f) {
        std::cerr << "unreachable solve failed: status=" << unreachable_status
                  << " residual=" << residual << "\n";
        return 1;
    }

    float degenerate[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    const float zero_lengths[] = {0, 0};
    const int32_t degenerate_status = fabrik_solve_f32(degenerate, 3, zero_lengths,
        target, 1, 0.00001f, 64, degenerate, nullptr);
    if (degenerate_status != FABRIK_DEGENERATE_CHAIN) {
        std::cerr << "degenerate solve failed: status=" << degenerate_status << "\n";
        return 1;
    }

    std::cout << "Fabrik C++ adapter smoke: PASS (status=" << status
              << ", measured=" << measured_status
              << ", unreachable=" << unreachable_status
              << ", degenerate=" << degenerate_status << ")\n";
    return 0;
}
