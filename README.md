# attitude-filter

estimate a spacecraft's orientation from a gyroscope, which measures rotation, and a star tracker, which measures orientation from stars. The filter also estimates gyro bias, the sensor's persistent offset. It can therefore track through a tracker outage without accumulating the full gyro-only drift.

the included experiment removes tracker readings from 80 to 120 seconds and adds one bad reading at 150 seconds. With seed 42, the final attitude error is **0.089°**, versus **18.57°** for the gyro alone. The filter rejects the bad reading. These are simulation results under the stated sensor model.

![Tracker outage experiment](docs/tracker-outage.png)

## build and run

requires C++20, CMake 3.20+, and Eigen. Install Eigen with `brew install eigen` on macOS or `sudo apt-get install libeigen3-dev` on Debian/Ubuntu.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 3
ctest --test-dir build --output-on-failure
./build/attitude-demo 42 > trace.csv
```

the demo writes a time series to standard output and a JSON summary to standard error. Change the seed to run the experiment with different sensor noise. Reproducing the same result also requires the same standard-library implementation of the random distributions.

## filter state

the orientation is a unit quaternion, a four-number rotation representation. The uncertainty state has six components: three small rotation errors and three gyro-bias errors. A three-component attitude error avoids treating the quaternion's four constrained coefficients as independent.

for each gyro sample, the filter subtracts the estimated bias and advances the quaternion. It advances the uncertainty with the matching linearized error model. A block-matrix exponential integrates process noise over that sample interval.

a tracker reading gives a rotation error relative to the current estimate. The filter compares that error with its predicted uncertainty before accepting it. An accepted reading corrects orientation and bias, then moves the covariance into the corrected attitude's local coordinate frame. A rejected reading leaves the estimate unchanged.

## checks and limits

the tests compare the transition and covariance-reset Jacobians with finite differences of real quaternion rotations. They check process-noise integration against a closed-form zero-rate solution and against two composed half steps. Twenty-four seeded simulations cover bias estimation, tracker outages, outliers, quaternion sign flips, covariance symmetry, and positive definiteness.

the filter assumes a reasonably close initial attitude and continuing gyro samples during a tracker outage. It does not recover initial attitude from star images, estimate clock offsets, or model tracker-alignment errors. The demo uses controlled Gaussian noise. Passing it does not establish performance on a real sensor.

the public interface is [filter.hpp](include/attitude/filter.hpp). [The equations and conventions](docs/filter.md) define the error frame and update. To create the plot, install Matplotlib and run `python examples/plot.py`. Enable sanitizers with `-DATTITUDE_SANITIZERS=ON -DCMAKE_BUILD_TYPE=Debug`.

the formulation follows Markley and Bauer's [Attitude Error Representations for Kalman Filtering](https://ntrs.nasa.gov/citations/20020060647).
