# Magnetometer Filter

| | |
|---|---|
| **Status** | Since v1.13 (#33, #36, #53). Proposed redesign for v1.17. |
| **Code** | `VehicleMagnetometer` in `src/modules/sensors/vehicle_magnetometer/`, `IST8310::RunImpl()` in `src/drivers/magnetometer/isentek/ist8310/IST8310.cpp` |

## Summary

Power lines add 50 Hz noise to the magnetometer.

Upstream PX4 samples the IST8310 too slowly to remove it, so it appears as a slow heading error.

This patch samples faster, low-pass filters, averages down to the publish rate and corrects the timestamp for the filter delay.

We assume the IST8310 is the primary magnetometer on the PX4.

## Why

### Sampling too slowly aliases the noise

Sampling at $f_s$ moves frequency $f$ to $|f - k\cdot f_s|$.

The upstream magnetometer driver runs at just under 50 Hz. This causes the 50 Hz noise to land close to DC, where it looks like a real heading change.

By increasing the sampling frequency to $f_s \approx 150$ Hz the 50 Hz noise stays at 50 Hz, well away from the signal, where it can be filtered out.

### The output is downsampled

The sensors module publishes `vehicle_magnetometer` to EKF2 at `SENS_MAG_RATE` = 15 Hz.

## Design

### 1. Sample at ~150 Hz

The IST8310 wait is reduced from 20 ms to 6 ms, so the sampling frequency is $f_s \approx 150$ Hz.

### 2. Low-pass filter

Two first-order low-pass filters in series. Both use a cut-off frequency $f_c =$ `SENS_MAG_LP_CUT` (default 10 Hz, 0 disables).

Implemented with `AlphaFilter::update(x, dt)` using the measured time between samples.

### 3. Average each publish window

The samples in each publish window are averaged, then one value is published.

The average's zeros at multiples of 15 Hz are exactly the frequencies that decimation moves onto DC.

Upstream decimation is unchanged. v1.13 replaced it with the latest filtered sample; v1.17 restores it.

### 4. Correct the timestamp

Upstream already accounts for the delay due to averaging.

We only need to subtract the remaining filter delay, which is $1/(\pi\cdot f_c)\approx32\,\text{ms}$ at 10 Hz.

EKF2_MAG_DELAY stays at 0, otherwise the delay is corrected twice.

## Notes for porting new PX4 Versions

The v1.13 warning and fallback to unfiltered data when sampling is slow (#33, #53) are not ported. The filters are stable at any sample rate.

#33 the `magnetometer_noise` is not ported; `sensor_mag` is logged at full rate instead.


