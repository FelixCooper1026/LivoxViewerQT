#pragma once

namespace fast_lio {

struct Pose6D {
    double offset_time = 0.0;
    double acc[3] = {};
    double gyr[3] = {};
    double vel[3] = {};
    double pos[3] = {};
    double rot[9] = {};
};

} // namespace fast_lio
