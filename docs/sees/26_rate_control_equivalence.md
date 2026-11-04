# Proposed Rate Controller

Our custom PX4 (PR #26) modifies the rate PID to include the filtered derivative of the target rate in the D term. This adds an extra "push" during sharp changes to the target rate, for example hard braking during obstacle avoidance.

This involved many changes to the rate controller.

This document shows the same behaviour can be achieved with vanilla PX4, retuned parameters and a single feed-forward term on the filtered derivative of the target attitude. Parameter values are at the end.

**Notation.**

- $\theta$ current attitude
- $\theta_{T}$ target attitude
- $\omega$ current rate
- $\omega_{T}$ target rate
- $e = \omega_{T} - \omega$ rate error
- $\tau$ normalised torque
- $s$ the Laplace variable: multiplying by $s$ differentiates, dividing by $s$ integrates. All
signals and filters are in the Laplace domain.
- Gains: $K_p$, $K_i$, $K_d$ from `MC_ROLLRATE_K/P/I/D` and $K_a$ = `MC_ROLL_P`.
- Filters, both first order:
  - $H(s) = \frac{\omega_h}{s+\omega_h}$, the IMU low pass filter: applied by PX4's sensors module to the angular acceleration, before it reaches the rate controller. Cut-off frequency $f_h =$ `IMU_DGYRO_CUTOFF`.
  - $F(s) = \frac{\omega_f}{s+\omega_f}$, the rate-controller D filter: added by PR #26 inside the rate controller and applied to the whole D term. Cut-off frequency $f_f =$ `MC_PR_D_CUTOFF`.
- $f$ frequency in Hz, so $\omega_h = 2\pi f_h$ and $\omega_f = 2\pi f_f$.
- A hat marks a vanilla quantity chosen to reproduce custom Sees, e.g. $\hat{K}_p$.

**Assumptions.**

- Small angles, so $s\theta \approx \omega$.
- Linear: target rate not clamped by `MC_ROLLRATE_MAX`, motors not saturated.
- Continuous time: at 400 Hz, discrete filters match their continuous forms below ~40 Hz.
- Roll and pitch only: yaw has $K_d = 0$, so PR #26 does nothing on yaw.

## Vanilla PX4 PID

A proportional-only controller is used to go from attitude to rate.

$$\omega_{T} = K_a\cdot (\theta_{T} - \theta) \tag{1}$$

A full PID controller is used for rate to torque.

$$\tau = \underbrace{K_p\cdot e}_{\text{Proportional}}
  + \underbrace{\frac{K_i}{s}\cdot e}_{\text{Integral}}
  - \underbrace{K_d\cdot H\cdot s\omega}_{\text{Derivative}} \tag{2}$$

## Custom Sees PID

PR #26 added a custom target rate derivative term to the PID.

$$\tau = K_p\cdot e + \frac{K_i}{s}\cdot e + K_d\cdot F\cdot(s\omega_{T} - H\cdot s\omega) \tag{3}$$

Differentiating (1):

$$s\omega_{T} = K_a\cdot (s\theta_{T} - \omega) \tag{4}$$

Substituting (4) into (3), with $\Delta K_p \equiv K_d\cdot K_a$:

$$\tau = \underbrace{K_p\cdot e}_{\text{Proportional}}
  + \underbrace{\frac{K_i}{s}\cdot e}_{\text{Integral}}
  + \underbrace{\Delta K_p\cdot F\cdot s\theta_{T} - \Delta K_p\cdot F\cdot\omega - K_d\cdot F\cdot H\cdot s\omega}_{\text{Derivative}} \tag{5}$$

## Equivalence

Comparing (2) and (5) term by term:

| Term | Vanilla PX4 PID | Custom Sees PID |
|---|---|---|
| Proportional | $K_p\cdot e$ | $K_p\cdot e$ |
| Integral | $\frac{K_i}{s}\cdot e$ | $\frac{K_i}{s}\cdot e$ |
| Derivative | $-K_d\cdot H\cdot s\omega$ | $-K_d\cdot F\cdot H\cdot s\omega - \Delta K_p\cdot F\cdot\omega + \Delta K_p\cdot F\cdot s\theta_{T}$ |

PR #26 only changes the derivative term. Its derivative term has three parts:

- the vanilla derivative filtered more heavily, $-K_d\cdot F\cdot H\cdot s\omega$
- a part proportional to the current rate, $-\Delta K_p\cdot F\cdot\omega$
- a feed-forward based on the derivative of target attitude, $\Delta K_p\cdot F\cdot s\theta_{T}$

We now show that vanilla PX4 can reproduce the first two by changing only the gain values (marked with a hat).

### Moving part of the D term into the P term

Since $F\cdot(s + \omega_f) = \omega_f$, the rate-controller D filter can be written as two terms:

$$F = 1 - \frac{s}{\omega_f}\cdot F \tag{6}$$

So, using $\Delta K_p = K_d\cdot K_a$, the part of the custom derivative term proportional to current rate can be split into

$$-\Delta K_p\cdot F\cdot\omega = -\Delta K_p\cdot\omega + K_d\cdot\frac{K_a}{\omega_f}\cdot F\cdot s\omega \tag{7}$$

Expanding the proportional term using (1):

$$K_p\cdot e = K_p\cdot K_a\cdot (\theta_{T} - \theta) - K_p\cdot\omega \tag{8}$$

Adding the first part of (7) to it:

$$K_p\cdot e - \Delta K_p\cdot\omega = K_p\cdot K_a\cdot (\theta_{T} - \theta) - (K_p + \Delta K_p)\cdot\omega \tag{9}$$

If we define our new gains as

$$\hat{K}_p = K_p + \Delta K_p \qquad \hat{K}_a = \frac{K_p\cdot K_a}{\hat{K}_p}$$

we can re-write (9) as a vanilla proportional term:

$$K_p\cdot e - \Delta K_p\cdot\omega = \hat{K}_p\cdot\big(\hat{K}_a\cdot(\theta_{T} - \theta) - \omega\big) \tag{10}$$

### The remaining D term

The second part of (7) joins the vanilla derivative.

$$-K_d\cdot F\cdot H\cdot s\omega + K_d\cdot\frac{K_a}{\omega_f}\cdot F\cdot s\omega$$

The second term has no $H$. But its magnitude is $K_a/\omega_f = 0.044$ times the first, so we approximate $H\approx1$.

$$-\hat{K}_d\cdot F\cdot H\cdot s\omega\tag{11}$$

where we get a new derivative gain defined as

$$\hat{K}_d = K_d\cdot\Big(1 - \frac{K_a}{\omega_f}\Big) $$

$F\cdot H$ is two first-order filters in series. Multiplying them out:

$$F\cdot H = \frac{\omega_f\cdot\omega_h}{(s+\omega_f)\cdot(s+\omega_h)} = \frac{1}{1 + s\cdot\Big(\frac{1}{\omega_f}+\frac{1}{\omega_h}\Big) + \frac{s^2}{\omega_f\cdot\omega_h}}$$

Below $\sqrt{f_f\cdot f_h} \approx 49$ Hz the $s^2$ term is small, so $F\cdot H$ is approximately a single first-order filter $\hat{H}$, whose time constant is the sum of the two:

$$\hat{H} = \frac{1}{1 + s/\hat{\omega}_h}, \qquad \frac{1}{\hat{\omega}_h} = \frac{1}{\omega_f} + \frac{1}{\omega_h} \tag{12}$$

## Proposed New PID Gains

| Term | Proposed Vanilla PX4 PID | Custom Sees PID |
|---|---|---|
| Proportional | $\hat{K}_p\cdot e$ | $K_p\cdot e$ |
| Integral | $\frac{K_i}{s}\cdot e$ | $\frac{K_i}{s}\cdot e$ |
| Derivative | $-\hat{K}_d\cdot \hat{H}\cdot s\omega$ | $-K_d\cdot F\cdot H\cdot s\omega - \Delta K_p\cdot F\cdot\omega + \Delta K_p\cdot F\cdot s\theta_{T}$ |

In the proposed vanilla column, $e$ is computed with $\hat{K}_a$ in (1).

### Adding a custom feed-forward term

Vanilla cannot reproduce the feed-forward term $\Delta K_p\cdot F\cdot s\theta_{T}$ which is driven by the rate of change of the target attitude.

So we add a custom feed-forward on the filtered derivative of the target attitude, with gain
$K_{ff}$ and filter $F$:

$$\tau = \hat{K}_p\cdot e + \frac{K_i}{s}\cdot e - \hat{K}_d\cdot\hat{H}\cdot s\omega + K_{ff}\cdot F\cdot s\theta_{T} \tag{13}$$

With $K_{ff} = \Delta K_p$ (13) reproduces (5) up to the approximations in (11) and
(12). Its gain is independent of the P and D gains, and with $K_{ff} = 0$ it is exactly vanilla.

Unlike PR #26, this term is zero whenever the target attitude is constant, so it only acts on commanded changes, not on the drone's own motion.

## Parameter Values

### Current

$K_a =$ `MC_ROLL_P` $=$ `MC_PITCH_P` $= 5.5$

$K_p =$ `MC_ROLLRATE_K` $\times$ `MC_ROLLRATE_P` = $0.08 \times 1.0 = 0.08$

$K_i =$ `MC_ROLLRATE_K` $\times$ `MC_ROLLRATE_I` = $0.08 \times 2.0 = 0.16$

$K_d =$ `MC_ROLLRATE_K` $\times$ `MC_ROLLRATE_D` = $0.08 \times 0.029 = 0.00232$

And filter cut-off frequencies of

$f_h =$ `IMU_DGYRO_CUTOFF` $= 120\,\text{Hz}$

$f_f =$ `MC_PR_D_CUTOFF` $= 20\,\text{Hz}$

### Proposed

$\Delta K_p = K_d\cdot K_a = 0.00232 \times 5.5 = 0.0128$

$\hat K_p = K_p + \Delta K_p = 0.0928$

$\hat K_a = K_p\cdot K_a/\hat K_p = 4.74$

$\hat K_d = K_d\cdot(1 - K_a/\omega_f) = 0.00232 \times 0.956 = 0.00222$

$K_{ff} = K_d\cdot K_a = 0.00232 \times 5.5 = 0.0128$

$\hat f_h = (1/20 + 1/120)^{-1} = 17$ Hz

## Other Changes

 PR #26 also changed how often the sensors module publishes vehicle_angular_velocity, from `IMU_GYRO_RATEMAX` (800 Hz) to `IMU_INTEG_RATE` (400 Hz).

The rate controller runs every time a new value for current angular velocity arrives. So the rate loop currently runs at 400 Hz.

Vanilla v1.17 publishes at `IMU_GYRO_RATEMAX`, so setting `IMU_GYRO_RATEMAX` $= 400$ keeps the 400 Hz loop with no code change.

**Yaw torque filter**. v1.17 adds a first-order low-pass on the yaw torque setpoint (`MC_YAW_TQ_CUTOFF`, default 2 Hz) that v1.13 does not have. Setting it to 0 matches v1.13, which the current yaw gains were tuned for.
