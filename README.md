# attitude-filter

Estimate a spacecraft's orientation from a gyroscope and occasional star-tracker readings. The filter also estimates gyro bias, so it can keep tracking during an observation outage without accumulating the full gyro-only drift.

The included experiment removes tracker readings from 80 to 120 seconds and injects one bad reading at 150 seconds. With seed 42, the final attitude error is **0.089°**, compared with **18.57°** using the gyro alone. The injected outlier is rejected. These are simulation results under the stated sensor model.

![Tracker outage experiment](docs/tracker-outage.png)

## Build and run

Requires C++20, CMake 3.20+, and Eigen. Install Eigen with `brew install eigen` on macOS or `sudo apt-get install libeigen3-dev` on Debian/Ubuntu.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 3
ctest --test-dir build --output-on-failure
./build/attitude-demo 42 > trace.csv
```

The demo writes a time series to stdout and a JSON summary to stderr. Change the seed to repeat the experiment with different sensor noise. Reproducibility assumes the same standard-library implementation of the random distributions.

## What the filter estimates

The orientation is a unit quaternion. The uncertainty has six components: three small rotation errors and three gyro-bias errors. Keeping a three-component attitude error avoids treating four constrained quaternion coefficients as independent variables.

A gyro sample advances the quaternion after subtracting the estimated bias. The covariance advances with the corresponding linearized error model. A block matrix exponential integrates the process noise over the sample interval.

A tracker observation provides a rotation error relative to the current estimate. The filter checks that error against its predicted uncertainty before accepting it. An accepted observation corrects orientation and bias. The covariance is then moved into the corrected attitude's local coordinate frame. A rejected observation leaves the estimate unchanged.

## What is checked

The tests compare the transition and covariance-reset Jacobians with finite differences of actual quaternion rotations. Process-noise integration is checked against a closed-form zero-rate solution and by composing two half steps. Twenty-four seeded simulations exercise bias estimation, tracker outages, outliers, quaternion sign flips, covariance symmetry, and positive definiteness.

The filter assumes the initial attitude is reasonably close and that gyro samples continue during a tracker outage. It does not solve initial attitude from star images, estimate clock offsets, or model tracker alignment errors. The demo's Gaussian noise is controlled; passing it does not establish performance on a real sensor.

The public interface is [filter.hpp](include/attitude/filter.hpp). [The equations and conventions](docs/filter.md) explain the error frame and update. For the plot, install Matplotlib and run `python examples/plot.py`. Sanitizers are enabled with `-DATTITUDE_SANITIZERS=ON -DCMAKE_BUILD_TYPE=Debug`.

The formulation follows the approach described by Markley and Bauer in [Attitude Error Representations for Kalman Filtering](https://ntrs.nasa.gov/citations/20020060647).
