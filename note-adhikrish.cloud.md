# attitude-filter

star trackers drop out during slews, sun exclusion and faults, and while they're unavailable a spacecraft has to estimate attitude from its gyros alone. gyros carry a persistent bias, and integrating it turns a small rate error into a growing attitude error. the way to survive an outage is to estimate the bias while the tracker is still available.

in this project's seeded scenario, gyro-only integration ends **18.57°** off and the filter ends **0.089°** off.

```text
gyro -> subtract bias -> predict attitude + covariance
                              |
    tracker -> gate on residual -> reject, or correct attitude + bias -> reset error frame
```

## design

it's a multiplicative extended kalman filter: a unit quaternion for attitude, gyro bias estimated alongside it, and a six-state error covariance (three rotation errors, three bias errors), so the four quaternion components are never treated as independent uncertain quantities.

the parts that usually go wrong are handled explicitly:

- prediction integrates the linearised dynamics and continuous process noise with a block matrix exponential, so results don't depend on the gyro sample rate.
- the measurement update uses a factored solve and the joseph form, with no explicit matrix inverse, to preserve symmetry and positive definiteness.
- after each accepted correction the covariance is transformed into the new error frame using the right jacobian of the rotation exponential. normalising the quaternion alone leaves the covariance expressed in the old frame.
- every update is computed as a complete candidate and validated before it replaces the stored state, so an invalid measurement or failed factorisation can't leave a partially updated filter.

the derivation is in [filter.md](docs/filter.md).

## results

a 300 s scenario with the tracker unavailable from 80 to 120 s and a false observation injected at 150 s, which the residual gate rejects. with seed 42 the final error is 0.089° against 18.57° for gyro-only integration. this is a controlled simulation, not flight data.

![Tracker outage experiment](docs/tracker-outage.png)

the [tests](tests/test_filter.cpp) compare the jacobians with finite differences, check the noise integration against a closed-form solution, and run 24 seeded scenarios covering bias estimation, outages, false readings, quaternion sign flips, and covariance symmetry and positive definiteness.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
ctest --test-dir build --output-on-failure
./build/attitude-demo 42 > trace.csv
```

requires eigen and a reasonable initial attitude. not modelled: lost-in-space initialisation, clock offsets, tracker misalignment, non-gaussian noise. the formulation follows markley and bauer, [attitude error representations for kalman filtering](https://ntrs.nasa.gov/citations/20020060647).
