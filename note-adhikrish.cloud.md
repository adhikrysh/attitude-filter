# attitude-filter

star trackers go blind sometimes (sun in the baffle, a slew, a fault), and while they're out the spacecraft points on gyro alone. the gyro has a small constant bias, and integrating a small constant error for long enough gets you somewhere very confident and very wrong.

in the seeded run, gyro-only ends up **18.57°** off. the filter ends at **0.089°**, because it learned the gyro's bias while the tracker could still see.

```text
gyro -> subtract bias -> predict attitude + covariance
                              |
         tracker -> gate on residual -> reject, or correct attitude + bias -> reset error frame
```

## how

it's a multiplicative EKF: a unit quaternion for attitude, bias estimated alongside it, and a 6-state error covariance (three rotation, three bias), so the four quaternion numbers never get treated as independent.

the fiddly bits are where filters usually break quietly. prediction integrates the dynamics and the continuous noise with a block matrix exponential, so the result doesn't change with your sample rate. the update uses Joseph form and a factored solve, no inverses. after every correction the covariance gets moved into the new error frame with the right Jacobian, because normalising the quaternion and calling it a day leaves the uncertainty pointing at the old frame. updates are built as a full candidate first, so a bad reading can't leave the filter half-updated. equations in [filter.md](docs/filter.md).

## results

300 s run, tracker blind from 80 to 120 s, one fake star reading at 150 s that gets rejected. seed 42: 0.089° vs 18.57°. simulation, not flight data.

![Tracker outage experiment](docs/tracker-outage.png)

[tests](tests/test_filter.cpp) check the Jacobians against finite differences, the noise integration against a closed form, and 24 seeded runs for bias, outages, fake readings, quaternion sign flips and a covariance that stays symmetric and positive definite.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
ctest --test-dir build --output-on-failure
./build/attitude-demo 42 > trace.csv
```

needs Eigen and a decent starting attitude. no lost-in-space solve, no clock offsets, no tracker misalignment, gaussian noise only. based on Markley and Bauer's [attitude error representations](https://ntrs.nasa.gov/citations/20020060647).
