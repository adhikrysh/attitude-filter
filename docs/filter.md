# Error state and covariance

The code uses Hamilton quaternions. `q * v` rotates a body-frame vector into the inertial frame. A body angular rate advances attitude by multiplying on the right: `q_next = q * Exp(rate * dt)`. Eigen stores quaternion coefficients as x, y, z, w, while its constructor takes w, x, y, z.

The error convention is `q_true = q_estimate * Exp(delta_angle)`. Bias error is true bias minus estimated bias. The six-component error vector combines these quantities.

For a constant corrected angular rate `w`, the linearized dynamics are:

```text
F = [ -skew(w)  -I ]
    [     0     0 ]
```

`skew(w) * v` is the cross product `w × v`. The continuous noise covariance is diagonal, with squared gyro-noise density in the attitude block and squared bias-walk density in the bias block. Noise densities use radians and seconds; the gyro noise supplied to a discrete sample has standard deviation `density / sqrt(dt)`.

The matrix exponential of `[[F,Q],[0,-F^T]] * dt` yields the state transition `Phi` in its upper-left block. Multiplying its upper-right block by `Phi^T` gives the integrated process covariance. Using the upper-right block alone would be incorrect.

A tracker quaternion produces the residual `Log(q_estimate^-1 * q_tracker)`. Its measurement Jacobian is `[I,0]`. The implementation solves the innovation system with a factorization rather than explicitly computing an inverse. It uses the Joseph covariance update to reduce numerical loss of symmetry and positive definiteness.

After applying the estimated rotation, the old local attitude-error coordinates are no longer centred at the new estimate. The covariance reset uses the right Jacobian of the rotation exponential, including the angle/bias cross terms. The finite-difference test checks this against `Log(Exp(-correction) * Exp(correction + perturbation))`.

Tracker covariance is a 3x3 matrix over small rotation errors in radians squared. It is not a matrix over four quaternion coefficients. Both input covariance and propagated covariance must remain positive definite. Invalid input or a failed factorization returns an error without applying a partial update.

The simulation's mean normalized squared error is checked with a deliberately broad regression threshold. That catches gross covariance mistakes. It is not a formal consistency study, and a small average attitude error alone would not establish correct uncertainty estimates.
