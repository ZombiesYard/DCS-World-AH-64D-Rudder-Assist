#pragma once

namespace autorudder {

struct CollectiveDriveConfig {
    double amplitude = 0.05;
    double period = 4.0;
    double settle_time = 6.0;
    double rate_limit = 0.40;
};

struct CollectiveDriveOutput {
    double collective = 0.0;
    double offset = 0.0;
    bool driving = false;
};

class CollectiveDrive {
public:
    explicit CollectiveDrive(CollectiveDriveConfig config);

    CollectiveDriveOutput update(double dt, double physical_collective, bool enabled);
    void reset();

private:
    CollectiveDriveConfig config_;
    bool has_output_ = false;
    double elapsed_ = 0.0;
    double output_collective_ = 0.0;
};

}  // namespace autorudder
